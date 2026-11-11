package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import java.util.List;

@TeleOp(name = "CodJava PIDF + 4 CRServo", group = "TeleOp")
public class CodJav extends OpMode {
    // 4 мотор — шасси (mecanum)
    // 1 мотор (ball)  — интейк (3 интейкті бір мотор айналдырады, допты шутерге де итереді)
    // 1 мотор (shuter) — шутер (PIDF)
    // CRServo: craneLeft + craneRight (кран), ballGate (допты ұстап тұратын қақпа), turret (турель)
    // TWO_GAMEPADS = false: бәрі бір геймпадта (gamepad1).
    // TWO_GAMEPADS = true:  gamepad1 = жүру/E-STOP, gamepad2 = қалғаны.
    //
    //   Жүру: оң стик = жүру, сол стик (солға/оңға) = бұрылу, L3 ұстап тұру = баяу режим
    //   Сұқ саусақпен (жиі):
    //     R1 = ату (қақпа ашылады, интейк допты шутерге итереді)
    //     L1 = жинауды қосу/өшіру
    //     L2 / R2 = турель солға / оңға
    //   Сирек:
    //     D-pad ↑ = шутер қосу, D-pad ↓ = шутер өшіру
    //     D-pad ← / → = шутер жылдамдығы -/+ (SPEED_STEP)
    //     ○ = жинау кері қосу/өшіру
    //     □ = шутер пресеті (NEAR → MID → FAR → NEAR ...)
    //     △ / ✕ = кранды көтеру / түсіру
    //     Touchpad (ұстап тұру) = шутерді КАЛИБРЛЕУ (тек шутер өшіп тұрғанда)
    //   Create + Options бірге = E-STOP
    private static final boolean TWO_GAMEPADS = false;

    private DcMotorEx[] drive;
    private DcMotorEx ball, shuter;
    private CRServo craneLeft, craneRight, ballGate, turret;
    private List<LynxModule> hubs;

    private final Gamepad prevDrv = new Gamepad(), curDrv = new Gamepad();
    private final Gamepad prevOp = new Gamepad(), curOp = new Gamepad();

    // Шутер PIDF: F = 32767 / (максималды ticks/s). Touchpad калибрлеуі өзі есептейді.
    private static final double P = 12, I = 0, D = 0, F = 13;
    // ШУТЕР ЖЫЛДАМДЫҒЫНЫҢ ПРЕСЕТТЕРІ (ticks/s). □ (Square) басқанда келесісіне ауысады.
    // Доп аса алысқа ұшса — сол пресеттің санын азайтыңыз, жетпесе — көбейтіңіз.
    private static final String[] PRESET_NAMES = {"NEAR", "MID", "FAR"};
    private static final double[] PRESET_SPEEDS = {1400, 1800, 2200};
    private static final int START_PRESET = 1;   // матч басында қай пресет (0=NEAR, 1=MID, 2=FAR)
    private static final double NOMINAL_VOLTAGE = 12.0;

    private static final double MIN_SPEED = 300, MAX_SPEED = 2500, SPEED_STEP = 50;
    private static final double READY_ERROR = 60, READY_SECONDS = 0.20;
    private static final double DRIVE_NORMAL = 1.0, DRIVE_SLOW = 0.35;

    // FTC-де стикті алға итергенде y теріс болады, сондықтан -1.
    // Егер робот алға итергенде артқа кетсе, мұны +1 қылыңыз.
    private static final double DRIVE_Y_SIGN = -1.0;
    private static final double[] DRIVE_TRIM = {1, 1, 1, 1};

    // CRServo бұрышы уақытпен есептеледі; DPS мәндерін өлшеп реттеңіз.
    private static final double CRANE_STEP_DEG = 30;
    private static final double CRANE_POWER = 0.30;
    private static final double CRANE_UP_DPS = 90, CRANE_DOWN_DPS = 90;
    private static final double TURRET_POWER = 0.25;
    private static final double GATE_DEG = 45, GATE_POWER = 0.30;
    private static final double GATE_OPEN_DPS = 90, GATE_CLOSE_DPS = 90;
    // Қақпа ашық тұратын уақыт: осы уақытта интейк допты шутерге итереді.
    // Көп доп өткізгіңіз келсе, көбейтіңіз.
    private static final double GATE_DWELL = 0.5, SHOT_WAIT_LIMIT = 3.0;

    private static final double CRANE_UP_TIME = CRANE_STEP_DEG / CRANE_UP_DPS;
    private static final double CRANE_DOWN_TIME = CRANE_STEP_DEG / CRANE_DOWN_DPS;
    private static final double GATE_OPEN_TIME = GATE_DEG / GATE_OPEN_DPS;
    private static final double GATE_CLOSE_TIME = GATE_DEG / GATE_CLOSE_DPS;
    private static final double MAX_CR_MOVE_SECONDS = 2.0;

    private static final double CRANE_LEFT_SIGN = 1, CRANE_RIGHT_SIGN = -1;
    private static final double TURRET_SIGN = 1, GATE_SIGN = 1;

    // Аккумулятор шектері (жүктеме астындағы кернеу, В)
    private static final double BATTERY_WARN_V = 11.0, BATTERY_CRITICAL_V = 10.0;
    private static final double BATTERY_WARN_SECONDS = 2.0, BATTERY_CRIT_SECONDS = 1.0;

    // ===== ҚУАТ БАСҚАРУ (аккумуляторды қорғау, brownout болдырмау) =====
    // Қуат біртіндеп өседі: 0 -> 1 өту уақыты = 1 / RAMP секунд. Тежеу кезінде шектеу жоқ.
    private static final double DRIVE_RAMP_PER_SEC = 10.0;    // шасси: 0.1 с (сезілмейді)
    private static final double INTAKE_RAMP_PER_SEC = 10.0;   // интейк: 0.1 с
    private static final double SHOOTER_RAMP_TPS2 = 5000;     // шутер: ticks/s өсу жылдамдығы (ticks/s²)
    // Жүру қуатын шектейтін қорғанысты толық өшіру үшін false қойыңыз.
    private static final boolean POWER_PROTECTION = true;
    // Кернеу LOAD_V_OK-тан төмен түссе, жүру қуаты азая бастайды; LOAD_V_MIN-де ең аз (LOAD_MIN_SCALE).
    private static final double LOAD_V_OK = 9.5, LOAD_V_MIN = 8.0, LOAD_MIN_SCALE = 0.6;
    // Хабтың жалпы тогы осыдан асса, жүру қуаты азаяды (сақтандырғыш 20 А болса, 16 А қалыпты).
    private static final double CURRENT_SOFT_LIMIT = 18.0;
    // Интейк қысылса (ток жоғары ұзақ тұрса), біраз уақыт тоқтайды.
    private static final double INTAKE_STALL_AMPS = 8.0, INTAKE_STALL_SECONDS = 0.4, INTAKE_STALL_PAUSE = 0.6;

    private final Pulse cranePulse = new Pulse();
    private double turretPower;
    private int intakeMode = 0;
    private enum GateState { CLOSED_ESTIMATE, WAIT_READY, OPENING, DWELL, CLOSING }
    private GateState gateState = GateState.CLOSED_ESTIMATE;
    private double gateDeadline, speed = PRESET_SPEEDS[START_PRESET], stableSince = -1;
    private double measuredSpeed, nextTelemetry;
    private boolean shooterOn, ready, wasReady, emergency;
    private final double[] presetSpeed = PRESET_SPEEDS.clone();
    private int presetIndex = START_PRESET;
    private String status = "Gate assumed CLOSED; calibrate CRServo timing";

    // PIDF калибрлеу және кернеу компенсациясы
    private double baseF = F, refVoltage = NOMINAL_VOLTAGE, lastCompVoltage = 0;
    private double nextCompCheck = 0, maxSpeedLimit = MAX_SPEED, calPeak = 0;
    private boolean calibrating = false;

    // Аккумулятор мониторы
    private double batteryVoltage = 0, minVoltage = Double.POSITIVE_INFINITY;
    private double nextVoltageRead = 0, nextCurrentRead = 0, nextBatteryWarn = 0;
    private double warnSince = -1, critSince = -1;
    private double hubCurrent, shooterCurrent, intakeCurrent, driveCurrent;
    private String batteryLevel = "NO DATA";

    // Қуат басқару күйі
    private double dt = 0.02, lastLoopTime = 0;
    private final double[] drivePrev = new double[4];
    private double intakeOut = 0, commandedSpeed = 0;
    private double powerScale = 1.0, powerTarget = 1.0;
    private double stallSince = -1, intakePauseUntil = 0;

    @Override public void init() {
        if (CRANE_UP_TIME > MAX_CR_MOVE_SECONDS || CRANE_DOWN_TIME > MAX_CR_MOVE_SECONDS
                || GATE_OPEN_TIME > MAX_CR_MOVE_SECONDS || GATE_CLOSE_TIME > MAX_CR_MOVE_SECONDS)
            throw new IllegalArgumentException("CRServo move exceeds time limit");

        drive = new DcMotorEx[] {
            hardwareMap.get(DcMotorEx.class, "mator1left"),
            hardwareMap.get(DcMotorEx.class, "matorLeft"),
            hardwareMap.get(DcMotorEx.class, "mator2right"),
            hardwareMap.get(DcMotorEx.class, "matorRight")
        };
        for (int n = 0; n < drive.length; n++) {
            drive[n].setDirection(n < 2 ? DcMotorSimple.Direction.REVERSE
                    : DcMotorSimple.Direction.FORWARD);
            drive[n].setPower(0);
            drive[n].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive[n].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        ball = hardwareMap.get(DcMotorEx.class, "ball");
        ball.setPower(0);
        ball.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ball.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shuter = hardwareMap.get(DcMotorEx.class, "shuter");
        shuter.setPower(0);
        shuter.setDirection(DcMotorSimple.Direction.FORWARD);
        shuter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shuter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shuter.setVelocityPIDFCoefficients(P, I, D, F);

        craneLeft = hardwareMap.get(CRServo.class, "craneLeft");
        craneRight = hardwareMap.get(CRServo.class, "craneRight");
        ballGate = hardwareMap.get(CRServo.class, "ballGate");
        turret = hardwareMap.get(CRServo.class, "turret");
        for (CRServo servo : new CRServo[]{craneLeft, craneRight, ballGate, turret}) {
            servo.setDirection(DcMotorSimple.Direction.FORWARD);
            servo.setPower(0);
        }
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        telemetry.addLine("Close gate manually BEFORE START. CR angles are estimates!");
        telemetry.addLine("E-STOP: Create + Options (latched); restart OpMode to reset.");
        telemetry.addData("Gamepads", TWO_GAMEPADS ? "gamepad1=drive, gamepad2=operator" : "gamepad1 only");
        telemetry.update();
    }

    @Override public void start() {
        lastLoopTime = getRuntime();
        curDrv.copy(gamepad1);
        prevDrv.copy(curDrv);
        curOp.copy(TWO_GAMEPADS ? gamepad2 : gamepad1);
        prevOp.copy(curOp);
    }

    @Override public void loop() {
        prevDrv.copy(curDrv);
        curDrv.copy(gamepad1);
        prevOp.copy(curOp);
        curOp.copy(TWO_GAMEPADS ? gamepad2 : gamepad1);
        double now = getRuntime();
        dt = Range.clip(now - lastLoopTime, 0, 0.1);
        lastLoopTime = now;

        if ((curDrv.back && curDrv.start) || (curOp.back && curOp.start)) emergency = true;
        if (emergency) {
            stopAll();
            telemetry.addLine("STOP LATCHED. Support crane, inspect gate, restart OpMode.");
            telemetry.update();
            return;
        }
        updateBattery(now);
        updateDrive();
        updateShooter(now);
        updateCraneAndTurret(now);
        updateGate(now);

        // L1 — жинау қосу/өшіру, ○ — кері қосу/өшіру (toggle).
        if (curOp.left_bumper && !prevOp.left_bumper) intakeMode = (intakeMode == 1) ? 0 : 1;
        if (curOp.b && !prevOp.b) intakeMode = (intakeMode == -1) ? 0 : -1;

        // Ату кезінде (қақпа ашық) интейк допты шутерге итереді.
        // Қақпа ашылып-жабылып жатқанда алға жинау тоқтап тұрады.
        double intakeTarget = intakeMode;
        if (gateState == GateState.DWELL && shooterOn) intakeTarget = 1;
        else if (intakeMode > 0 && gateState != GateState.CLOSED_ESTIMATE) intakeTarget = 0;
        if (now < intakePauseUntil) intakeTarget = 0;   // қысылудан қорғау
        intakeOut = ramp(intakeOut, intakeTarget, INTAKE_RAMP_PER_SEC * dt);
        ball.setPower(intakeOut);
        double intakePower = intakeOut;

        if (now >= nextTelemetry) {
            nextTelemetry = now + 0.1;
            telemetry.addData("Shooter / preset", "%s / %s", shooterOn ? "ON" : "OFF", PRESET_NAMES[presetIndex]);
            telemetry.addData("Target / actual ticks/s", "%.0f / %.0f", shooterOn ? speed : 0, measuredSpeed);
            telemetry.addData("READY", ready);
            telemetry.addData("Shooter F / max limit", "%.2f / %.0f", baseF, maxSpeedLimit);
            telemetry.addData("Intake command / power", "%d / %.1f", intakeMode, intakePower);
            telemetry.addData("Gate (estimated)", gateState);
            telemetry.addData("Crane / turret moving", "%s / %s", cranePulse.busy, turretPower != 0);
            telemetry.addData("Status", status);
            telemetry.addData("BATTERY", "%.2f V (min %.2f) %s", batteryVoltage,
                    Double.isFinite(minVoltage) ? minVoltage : 0.0, batteryLevel);
            telemetry.addData("Current A", "hub %.1f | shooter %.1f | intake %.1f | drive %.1f",
                    hubCurrent, shooterCurrent, intakeCurrent, driveCurrent);
            telemetry.addData("Drive power limit", "%.0f%%%s", powerScale * 100,
                    powerScale < 0.95 ? "  (LOAD PROTECTION)" : "");
            telemetry.update();
        }
    }

    // Аккумулятор мониторы: кернеу, минимум, ток (қай мотор көп тартады) және ескерту.
    private void updateBattery(double now) {
        if (now >= nextVoltageRead) {
            nextVoltageRead = now + 0.1;
            double v = Double.POSITIVE_INFINITY;
            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                double x = sensor.getVoltage();
                if (x > 6) v = Math.min(v, x);
            }
            if (Double.isFinite(v)) {
                batteryVoltage = v;
                minVoltage = Math.min(minVoltage, v);
                if (v < BATTERY_WARN_V) { if (warnSince < 0) warnSince = now; } else warnSince = -1;
                if (v < BATTERY_CRITICAL_V) { if (critSince < 0) critSince = now; } else critSince = -1;
                if (critSince >= 0 && now - critSince >= BATTERY_CRIT_SECONDS) batteryLevel = "CRITICAL!";
                else if (warnSince >= 0 && now - warnSince >= BATTERY_WARN_SECONDS) batteryLevel = "LOW";
                else batteryLevel = "OK";
            }
        }
        // Ток оқу баяу болуы мүмкін, сондықтан сирек (0.25 с).
        if (now >= nextCurrentRead) {
            nextCurrentRead = now + 0.25;
            double total = 0;
            for (LynxModule hub : hubs) total += hub.getCurrent(CurrentUnit.AMPS);
            hubCurrent = total;
            shooterCurrent = shuter.getCurrent(CurrentUnit.AMPS);
            intakeCurrent = ball.getCurrent(CurrentUnit.AMPS);
            double d = 0;
            for (DcMotorEx motor : drive) d += motor.getCurrent(CurrentUnit.AMPS);
            driveCurrent = d;
        }
        // Интейк қысылу қорғанысы: ток ұзақ уақыт жоғары болса, интейк біраз тоқтайды.
        if (Math.abs(intakeOut) > 0.3 && intakeCurrent > INTAKE_STALL_AMPS) {
            if (stallSince < 0) stallSince = now;
            if (now - stallSince >= INTAKE_STALL_SECONDS) {
                intakePauseUntil = now + INTAKE_STALL_PAUSE;
                stallSince = -1;
                status = "Intake stalled: paused briefly (check jam)";
            }
        } else stallSince = -1;

        // Жүру қуатын шектеу: кернеу төмендесе немесе жалпы ток көп болса.
        // Шутер басымдықта қалады (оның оборотына тимейміз), тек шасси азаяды.
        if (POWER_PROTECTION && batteryVoltage > 6) {
            double vScale = LOAD_MIN_SCALE + (1 - LOAD_MIN_SCALE)
                    * (batteryVoltage - LOAD_V_MIN) / (LOAD_V_OK - LOAD_V_MIN);
            vScale = Range.clip(vScale, LOAD_MIN_SCALE, 1.0);
            double cScale = hubCurrent > CURRENT_SOFT_LIMIT
                    ? Range.clip(CURRENT_SOFT_LIMIT / hubCurrent, LOAD_MIN_SCALE, 1.0) : 1.0;
            powerTarget = Math.min(vScale, cScale);
        } else powerTarget = 1.0;
        // Азаю тез (4/с), қалпына келу баяу (0.5/с), сонда қуат тербелмейді.
        double rate = powerTarget < powerScale ? 4.0 : 0.5;
        powerScale += Range.clip(powerTarget - powerScale, -rate * dt, rate * dt);

        // Геймпад дірілі: LOW — 2 блип, CRITICAL — 4 блип (әр 8 секунд сайын).
        if (!batteryLevel.equals("OK") && !batteryLevel.equals("NO DATA") && now >= nextBatteryWarn) {
            nextBatteryWarn = now + 8;
            gamepad1.rumbleBlips(batteryLevel.startsWith("CRIT") ? 4 : 2);
        }
    }

    private void updateDrive() {
        // Оң стик — жүру; сол стик — бұрылу.
        double y = shaped(curDrv.right_stick_y) * DRIVE_Y_SIGN;
        double x = shaped(curDrv.right_stick_x);
        double turn = shaped(curDrv.left_stick_x);
        double[] powers = {y + x + turn, y - x + turn, y - x - turn, y + x - turn};
        double max = 1;
        for (int i = 0; i < 4; i++) {
            powers[i] *= DRIVE_TRIM[i];
            max = Math.max(max, Math.abs(powers[i]));
        }
        // Баяу режим: сол стикті басып ұстап тұру.
        double scale = (curDrv.left_stick_button ? DRIVE_SLOW : DRIVE_NORMAL) * powerScale;
        for (int i = 0; i < 4; i++) {
            drivePrev[i] = ramp(drivePrev[i], powers[i] / max * scale, DRIVE_RAMP_PER_SEC * dt);
            drive[i].setPower(drivePrev[i]);
        }
    }

    private void updateShooter(double now) {
        // Калибрлеу: Touchpad ұстап тұрғанда шутер толық қуатпен айналып, максималды
        // жылдамдық өлшенеді. Жібергенде F = 32767 / максимум автоматты есептеледі.
        boolean canCalibrate = !shooterOn && gateState == GateState.CLOSED_ESTIMATE;
        if (curOp.touchpad && canCalibrate) {
            if (!calibrating) {
                calibrating = true;
                calPeak = 0;
                shuter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            shuter.setPower(1.0);
            measuredSpeed = shuter.getVelocity();
            calPeak = Math.max(calPeak, measuredSpeed);
            ready = false;
            wasReady = false;
            stableSince = -1;
            status = String.format("CALIBRATING: peak %.0f ticks/s (release Touchpad to apply)", calPeak);
            return;
        }
        if (calibrating) finishCalibration();

        // Кернеуге байланысты F түзету: аккумулятор отырғанда да оборот сақталады.
        if (now >= nextCompCheck) {
            nextCompCheck = now + 0.25;
            double v = batteryVoltage;
            if (v > 6 && Math.abs(v - lastCompVoltage) > 0.2) {
                lastCompVoltage = v;
                shuter.setVelocityPIDFCoefficients(P, I, D, baseF * refVoltage / v);
            }
        }

        double oldSpeed = speed;
        // □ — келесі пресетке ауысу (NEAR / MID / FAR).
        if (curOp.x && !prevOp.x) {
            presetIndex = (presetIndex + 1) % PRESET_SPEEDS.length;
            speed = presetSpeed[presetIndex];
        }
        // D-pad ← / → — ағымдағы пресетті 50-ге дәлдеу (келесіде ауысқанда да сақталады).
        if (curOp.dpad_left && !prevOp.dpad_left) speed -= SPEED_STEP;
        if (curOp.dpad_right && !prevOp.dpad_right) speed += SPEED_STEP;
        speed = Range.clip(speed, MIN_SPEED, maxSpeedLimit);
        presetSpeed[presetIndex] = speed;
        if (speed != oldSpeed) {
            stableSince = -1;
            ready = false;
        }
        if (curOp.dpad_up && !prevOp.dpad_up) shooterOn = true;
        if (curOp.dpad_down && !prevOp.dpad_down) shooterOn = false;
        // Лақтыру кезінде де PIDF қосулы: мақсатты жылдамдық өзгермейді.
        // Шутер біртіндеп жылдамдайды (ток соққысы болмайды), баяулату тез.
        if (shooterOn) {
            if (commandedSpeed < speed)
                commandedSpeed = Math.min(speed, commandedSpeed + SHOOTER_RAMP_TPS2 * dt);
            else commandedSpeed = speed;
            shuter.setVelocity(commandedSpeed);
        } else {
            commandedSpeed = Math.max(0, measuredSpeed);
            shuter.setPower(0);
        }
        measuredSpeed = shuter.getVelocity();
        // Ату басталардан бұрын жылдамдық рұқсат етілген аралықта тұрақтануы керек.
        if (shooterOn && Math.abs(measuredSpeed - speed) <= READY_ERROR) {
            if (stableSince < 0) stableSince = now;
            ready = now - stableSince >= READY_SECONDS;
        } else {
            stableSince = -1;
            ready = false;
        }
        if (ready && !wasReady) (TWO_GAMEPADS ? gamepad2 : gamepad1).rumble(150);
        wasReady = ready;
    }

    private void finishCalibration() {
        calibrating = false;
        shuter.setPower(0);
        shuter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (calPeak > 500) {
            baseF = 32767.0 / calPeak;
            refVoltage = batteryVoltage > 6 ? batteryVoltage : NOMINAL_VOLTAGE;
            maxSpeedLimit = Math.min(MAX_SPEED, 0.9 * calPeak);
            status = String.format("Calibrated: F=%.2f, max=%.0f ticks/s, limit=%.0f", baseF, calPeak, maxSpeedLimit);
        } else {
            status = "Calibration failed: peak too low (hold Touchpad ~3 s)";
        }
        shuter.setVelocityPIDFCoefficients(P, I, D, baseF);
        lastCompVoltage = 0;
        nextCompCheck = 0;
    }

    private void updateCraneAndTurret(double now) {
        cranePulse.update(now);
        boolean gateIdle = gateState == GateState.CLOSED_ESTIMATE;
        if (gateIdle && !cranePulse.busy) {
            if (curOp.y && !prevOp.y && !curOp.a)
                cranePulse.begin(CRANE_POWER, CRANE_UP_TIME, now);
            else if (curOp.a && !prevOp.a && !curOp.y)
                cranePulse.begin(-CRANE_POWER, CRANE_DOWN_TIME, now);
        }

        // L2/R2 — турель; жібергенде немесе бірге басқанда тоқтайды.
        boolean left = curOp.left_trigger > 0.1;
        boolean right = curOp.right_trigger > 0.1;
        turretPower = 0;
        if (gateIdle && left != right) {
            turretPower = TURRET_POWER * (right
                    ? (curOp.right_trigger - 0.1) / 0.9
                    : -(curOp.left_trigger - 0.1) / 0.9);
        }
        craneLeft.setPower(cranePulse.power * CRANE_LEFT_SIGN);
        craneRight.setPower(cranePulse.power * CRANE_RIGHT_SIGN);
        turret.setPower(turretPower * TURRET_SIGN);
    }

    private void updateGate(double now) {
        // R1 — шутер дайындығын күту, қақпаны ашу, интейкпен беру, қақпаны жабу.
        boolean request = curOp.right_bumper && !prevOp.right_bumper;
        if (request && gateState == GateState.CLOSED_ESTIMATE) {
            if (shooterOn && !cranePulse.busy && turretPower == 0) {
                gateState = GateState.WAIT_READY;
                gateDeadline = now + SHOT_WAIT_LIMIT;
                status = "Waiting for shooter";
            } else status = "Start shooter; finish crane/turret movement first";
        }
        switch (gateState) {
            case CLOSED_ESTIMATE:
                ballGate.setPower(0);
                break;
            case WAIT_READY:
                ballGate.setPower(0);
                if (!shooterOn || now >= gateDeadline) {
                    gateState = GateState.CLOSED_ESTIMATE;
                    status = "Feed cancelled / readiness timeout";
                } else if (ready) {
                    gateState = GateState.OPENING;
                    gateDeadline = now + GATE_OPEN_TIME;
                    ballGate.setPower(GATE_POWER * GATE_SIGN);
                }
                break;
            case OPENING:
                if (now >= gateDeadline) {
                    ballGate.setPower(0);
                    if (!ready) beginClose(now);
                    else {
                        gateState = GateState.DWELL;
                        gateDeadline = now + GATE_DWELL;
                    }
                }
                break;
            case DWELL:
                // Доп өткенде оборот сәл түседі: DWELL уақыты біткенше қақпа ашық тұрады.
                if (!shooterOn || now >= gateDeadline) beginClose(now);
                break;
            case CLOSING:
                if (now >= gateDeadline) {
                    ballGate.setPower(0);
                    gateState = GateState.CLOSED_ESTIMATE;
                    stableSince = -1;
                    ready = false;
                    status = "Cycle finished; gate closed by timing only";
                }
                break;
        }
    }

    private void beginClose(double now) {
        gateState = GateState.CLOSING;
        gateDeadline = now + GATE_CLOSE_TIME;
        ballGate.setPower(-GATE_POWER * GATE_SIGN);
    }

    // Қуат тек біртіндеп өседі (step), ал азайғанда бірден түседі (тежеу).
    private static double ramp(double prev, double target, double step) {
        if (Math.abs(target) < Math.abs(prev) && target * prev >= 0) return target;
        return prev + Range.clip(target - prev, -step, step);
    }

    private static double shaped(double input) {
        double deadband = 0.05;
        if (Math.abs(input) <= deadband) return 0;
        double a = (Math.abs(input) - deadband) / (1 - deadband);
        return Math.copySign(0.65 * a + 0.35 * a * a * a, input);
    }

    private static class Pulse {
        boolean busy;
        double power, deadline;
        void begin(double requestedPower, double seconds, double now) {
            power = requestedPower;
            deadline = now + seconds;
            busy = true;
        }
        void update(double now) { if (busy && now >= deadline) cancel(); }
        void cancel() { busy = false; power = 0; }
    }

    private void stopAll() {
        shooterOn = false;
        intakeMode = 0;
        ready = false;
        stableSince = -1;
        cranePulse.cancel();
        turretPower = 0;
        if (calibrating && shuter != null) shuter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        calibrating = false;
        intakeOut = 0;
        commandedSpeed = 0;
        for (int i = 0; i < drivePrev.length; i++) drivePrev[i] = 0;
        if (drive != null) for (DcMotor motor : drive) if (motor != null) motor.setPower(0);
        if (ball != null) ball.setPower(0);
        if (shuter != null) shuter.setPower(0);
        for (CRServo servo : new CRServo[]{craneLeft, craneRight, ballGate, turret})
            if (servo != null) servo.setPower(0);
    }

    @Override public void stop() { stopAll(); }
}
