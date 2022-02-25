package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * A class that applies a given percentage sensitivity to an input for output.
 * Controls how sensitive the output is given the input.
 */
public class SensitivityController implements Sendable {
    private double positivePercentageSensitivity;
    private double negativePercentageSensitivity;

    private double positiveSensitivityExponent;
    private double negativeSensitivityExponent;

    private double positiveMinimumValue;
    private double negativeMinimumValue;

    private double positiveMaximumValue;
    private double negativeMaximumValue;

    private SensitivityController(Builder builder) {
        this.positivePercentageSensitivity = builder.positivePercentageSensitivity;
        this.negativePercentageSensitivity = builder.negativePercentageSensitivity;
        this.positiveMinimumValue = builder.positiveMinimumValue;
        this.negativeMinimumValue = builder.negativeMinimumValue;
        this.positiveMaximumValue = builder.positiveMaximumValue;
        this.negativeMaximumValue = builder.negativeMaximumValue;

        recalculateSensitivityExponents();
    }

    /**
     * Constructor with sensitivity arguments.
     *
     * @param positivePercentageSensitivity The positive input region's output sensitivity
     * @param negativePercentageSensitivity The negative input region's output sensitivity
     */
    public SensitivityController(double positivePercentageSensitivity, double negativePercentageSensitivity) {
        this.positivePercentageSensitivity = DreadbotMath.clampValue(positivePercentageSensitivity, -100.0d, 100.0d);
        this.negativePercentageSensitivity = DreadbotMath.clampValue(negativePercentageSensitivity, -100.0d, 100.0d);

        recalculateSensitivityExponents();
    }

    /**
     * Filters the input to apply a sensitivity to the output.
     *
     * @param input The joystick input
     * @return The filtered output
     */
    public double calculate(double input) {
        input = DreadbotMath.clampValue(input, -1.0d, 1.0d);

        System.out.println("positiveMaximumValue = " + positiveMaximumValue);
        System.out.println("negativeMaximumValue = " + negativeMaximumValue);

        if(input >= 0.0d) return positiveMaximumValue * Math.pow(input, positiveSensitivityExponent);
        return negativeMaximumValue * -Math.pow(-input, negativeSensitivityExponent);
    }

    private void recalculateSensitivityExponents() {
        positiveSensitivityExponent = positivePercentageSensitivity / 100.0d;
        positiveSensitivityExponent = Math.pow(10.0d, positiveSensitivityExponent);
        positiveSensitivityExponent = 1.0d / positiveSensitivityExponent;

        negativeSensitivityExponent = negativePercentageSensitivity / 100.0d;
        negativeSensitivityExponent = Math.pow(10.0d, negativeSensitivityExponent);
        negativeSensitivityExponent = 1.0d / negativeSensitivityExponent;
    }

    /**
     * Initializes this {@link Sendable} object.
     *
     * @param builder sendable builder
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SensitivityController");
        builder.addDoubleProperty("positivePercentageSensitivity", this::getPositivePercentageSensitivity, this::setPositivePercentageSensitivity);
        builder.addDoubleProperty("negativePercentageSensitivity", this::getNegativePercentageSensitivity, this::setNegativePercentageSensitivity);
        builder.addDoubleProperty("positiveMinimumValue", this::getPositiveMinimumValue, this::setPositiveMinimumValue);
        builder.addDoubleProperty("negativeMinimumValue", this::getNegativeMinimumValue, this::setNegativeMinimumValue);
        builder.addDoubleProperty("positiveMaximumValue", this::getPositiveMaximumValue, this::setPositiveMaximumValue);
        builder.addDoubleProperty("negativeMaximumValue", this::getNegativeMaximumValue, this::setNegativeMaximumValue);
    }

    public static class Builder {
        private final double positivePercentageSensitivity;
        private final double negativePercentageSensitivity;

        private double positiveMinimumValue = 0.0d;
        private double negativeMinimumValue = 0.0d;

        private double positiveMaximumValue = 1.0d;
        private double negativeMaximumValue = 1.0d;

        public Builder(double positivePercentageSensitivity, double negativePercentageSensitivity) {
            this.positivePercentageSensitivity = DreadbotMath.clampValue(positivePercentageSensitivity, -100.0d, 100.0d);
            this.negativePercentageSensitivity = DreadbotMath.clampValue(negativePercentageSensitivity, -100.0d, 100.0d);
        }

        public Builder minimumValues(double positiveMinimumValue, double negativeMinimumValue) {
            this.positiveMinimumValue = DreadbotMath.clampValue(positiveMinimumValue, 0.0d, 1.0d);
            this.negativeMinimumValue = DreadbotMath.clampValue(negativeMinimumValue, 0.0d, 1.0d);
            return this;
        }

        public Builder maximumValues(double positiveMaximumValue, double negativeMaximumValue) {
            this.positiveMaximumValue = DreadbotMath.clampValue(positiveMaximumValue, 0.0d, 1.0d);
            this.negativeMaximumValue = DreadbotMath.clampValue(negativeMaximumValue, 0.0d, 1.0d);
            return this;
        }

        public SensitivityController build() {
            SensitivityController sensitivityController = new SensitivityController(this);
            return sensitivityController;
        }
    }

    public double getPositivePercentageSensitivity() {
        return positivePercentageSensitivity;
    }

    public void setPositivePercentageSensitivity(double positivePercentageSensitivity) {
        this.positivePercentageSensitivity = positivePercentageSensitivity;
    }

    public double getNegativePercentageSensitivity() {
        return negativePercentageSensitivity;
    }

    public void setNegativePercentageSensitivity(double negativePercentageSensitivity) {
        this.negativePercentageSensitivity = negativePercentageSensitivity;
    }

    public double getPositiveMinimumValue() {
        return positiveMinimumValue;
    }

    public void setPositiveMinimumValue(double positiveMinimumValue) {
        this.positiveMinimumValue = positiveMinimumValue;
    }

    public double getNegativeMinimumValue() {
        return negativeMinimumValue;
    }

    public void setNegativeMinimumValue(double negativeMinimumValue) {
        this.negativeMinimumValue = negativeMinimumValue;
    }

    public double getPositiveMaximumValue() {
        return positiveMaximumValue;
    }

    public void setPositiveMaximumValue(double positiveMaximumValue) {
        this.positiveMaximumValue = positiveMaximumValue;
    }

    public double getNegativeMaximumValue() {
        return negativeMaximumValue;
    }

    public void setNegativeMaximumValue(double negativeMaximumValue) {
        this.negativeMaximumValue = negativeMaximumValue;
    }
}
