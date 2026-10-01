package frc.robot.util.motor;

import java.util.Map;
import java.util.function.DoubleConsumer;

import frc.robot.util.TunableNumber;

final class MotorTuning {
    private MotorTuning() {}

    static void register(MotorConfig config, Map<String, DoubleConsumer> edits) {
        config.sources.forEach((gain, source) -> {
            DoubleConsumer edit = edits.get(gain);
            if (edit == null) {
                return;
            }
            TunableNumber tunable = new TunableNumber(config.name + "/" + gain, config.value(gain), source);
            if (gain.equals("Current Limit")) {
                tunable.integer();
            }
            tunable.onChange(edit);
        });
    }
}
