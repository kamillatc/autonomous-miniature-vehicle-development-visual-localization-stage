#!/usr/bin/env python3
"""
pid.py

Classes de controle reutilizáveis pelo pacote carrinho_control.
Mantidas separadas dos nodes pra poder ser testadas isoladamente
(sem precisar subir rclpy) e reaproveitadas por qualquer node novo
de controle que você criar (ex: pid_velocity_node, pid_steering_node).
"""


class PID:
    """PID com anti-windup (clamping) e filtro passa-baixa no termo derivativo."""

    def __init__(self, kp, ki, kd, out_min, out_max, d_filter_alpha=0.2):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.d_filter_alpha = d_filter_alpha

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_deriv_filtered = 0.0

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_deriv_filtered = 0.0

    def update(self, error, dt):
        if dt <= 0.0:
            return 0.0

        raw_deriv = (error - self._prev_error) / dt
        deriv_filtered = (
            self.d_filter_alpha * self._prev_deriv_filtered
            + (1.0 - self.d_filter_alpha) * raw_deriv
        )

        tentative_integral = self._integral + error * dt
        p_term = self.kp * error
        i_term = self.ki * tentative_integral
        d_term = self.kd * deriv_filtered
        output_unclamped = p_term + i_term + d_term

        if self.out_min <= output_unclamped <= self.out_max:
            self._integral = tentative_integral  # anti-windup por clamping

        i_term = self.ki * self._integral
        output = p_term + i_term + d_term
        output = max(self.out_min, min(self.out_max, output))

        self._prev_error = error
        self._prev_deriv_filtered = deriv_filtered
        return output


class LowPassFilter:
    """Filtro passa-baixa exponencial simples (EMA), usado no giro cru da IMU."""

    def __init__(self, alpha=0.2):
        self.alpha = alpha  # quanto MENOR, mais suave (mais peso no histórico)
        self._value = None

    def update(self, new_value):
        if self._value is None:
            self._value = new_value
        else:
            self._value = self.alpha * new_value + (1.0 - self.alpha) * self._value
        return self._value
