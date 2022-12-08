// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.filter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.util.WPIUtilJNI;

/**
 * A class that limits the rate of change of an input value. Useful for implementing voltage,
 * setpoint, and/or output ramps. A slew-rate limit is most appropriate when the quantity being
 * controlled is a velocity or a voltage; when controlling a position, consider using a {@link
 * edu.wpi.first.math.trajectory.TrapezoidProfile} instead.
 */
public class SlewRateLimiter2d {
  private final double m_rateLimit;
  private Matrix<N2, N1> m_prevVal;
  private double m_prevTime;

  /**
   * Creates a new SlewRateLimiter with the given rate limit and initial value.
   *
   * @param rateLimit The rate-of-change limit, in units per second.
   * @param initialValue The initial value of the input.
   */
  public SlewRateLimiter2d(double rateLimit, Vector<N2> initialValue) {
    m_rateLimit = rateLimit;
    m_prevVal = initialValue;
    m_prevTime = WPIUtilJNI.now() * 1e-6;
  }

  /**
   * Creates a new SlewRateLimiter with the given rate limit and an initial value of zeroes.
   *
   * @param rateLimit The rate-of-change limit, in units per second.
   */
  public SlewRateLimiter2d(double rateLimit) {
    Matrix<N2, N1> initialValue = new Matrix<N2, N1>(Nat.N2(), Nat.N1());
    initialValue.fill(0);
    m_rateLimit = rateLimit;
    m_prevVal = initialValue;
    m_prevTime = WPIUtilJNI.now() * 1e-6;
  }

  /**
   * Filters the input to limit its slew rate.
   *
   * @param input The input value whose slew rate is to be limited.
   * @return The filtered value, which will not change faster than the slew rate.
   */
  public Matrix<N2, N1> calculate(Matrix<N2, N1> input) {
    double currentTime = WPIUtilJNI.now() * 1e-6;
    double dt = currentTime - m_prevTime;
    Matrix<N2, N1> difference = input.minus(m_prevVal);
    double norm = difference.normF();
    // m_rateLimit * dt will be the maximum it's allowed to change
    m_prevVal  = m_prevVal.plus(difference.times(Math.min(1, (m_rateLimit * dt)/norm)));
    m_prevTime = currentTime;
    return m_prevVal;
  }

  public State calculate(double x, double y) {
    Matrix<N2, N1> output = calculate(VecBuilder.fill(x, y));
    State outputState = new State(output.get(0, 0), output.get(1, 0));
    return outputState;
  }

  /**
   * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
   *
   * @param value The value to reset to.
   */
  public void reset(Matrix<N2, N1> value) {
    m_prevVal = value;
    m_prevTime = WPIUtilJNI.now() * 1e-6;
  }

  public class State {
    public double x;
    public double y;
    public State(double x, double y) {
      this.x = x;
      this.y = y;
    }
  }
}
