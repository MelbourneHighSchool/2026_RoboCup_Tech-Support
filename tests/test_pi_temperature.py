from lib.pi_temperature import PiTemperature


def test_temperature_conversion_cache_and_sensor_failure(tmp_path):
    sensor = tmp_path / "temp"
    sensor.write_text("54321\n")
    temperature = PiTemperature(sensor)
    temperature._sample()
    assert temperature.celsius == 54.321
    sensor.write_text("60000\n")
    # Getting a cached value does not read the sensor again.
    assert temperature.celsius == 54.321
    temperature._sample()
    assert temperature.celsius == 60.0
    sensor.write_text("invalid")
    temperature._sample()
    assert temperature.celsius is None
    sensor.unlink()
    temperature._sample()
    assert temperature.celsius is None
