from dataclasses import dataclass


@dataclass(frozen=True)
class AlarmLimits:
    hr_low: int = 50
    hr_high: int = 120
    resp_low: int = 8
    resp_high: int = 30
    spo2_low: int = 90


@dataclass(frozen=True)
class AlarmResult:
    alarms: list
    hr_alarm: bool
    resp_alarm: bool
    spo2_alarm: bool


def evaluate_alarm_state(hr, resp_rate, spo2, lead_status, limits):
    alarms = []
    hr_alarm = False
    resp_alarm = False
    spo2_alarm = False

    if hr is not None:
        if hr < limits.hr_low:
            alarms.append(f"\u5fc3\u7387\u8fc7\u4f4e {hr}")
            hr_alarm = True
        elif hr > limits.hr_high:
            alarms.append(f"\u5fc3\u7387\u8fc7\u9ad8 {hr}")
            hr_alarm = True

    if resp_rate is not None:
        if resp_rate < limits.resp_low:
            alarms.append(f"\u547c\u5438\u8fc7\u4f4e {resp_rate}")
            resp_alarm = True
        elif resp_rate > limits.resp_high:
            alarms.append(f"\u547c\u5438\u8fc7\u9ad8 {resp_rate}")
            resp_alarm = True

    if spo2 is not None and spo2 < limits.spo2_low:
        alarms.append(f"\u8840\u6c27\u8fc7\u4f4e {spo2}%")
        spo2_alarm = True

    for name, ok in lead_status.items():
        if ok is False:
            alarms.append(f"{name}\u5bfc\u8054\u5f02\u5e38")

    return AlarmResult(alarms, hr_alarm, resp_alarm, spo2_alarm)
