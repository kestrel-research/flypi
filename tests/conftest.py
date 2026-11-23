from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))

from fake_vehicle import FakeVehicle, free_udp_port  # noqa: E402

import flypi  # noqa: E402


@pytest.fixture
def make_vehicle():
    """Start a FakeVehicle; returns (vehicle, address) and cleans up afterwards."""
    started: list[FakeVehicle] = []

    def factory(**kwargs):
        port = free_udp_port()
        vehicle = FakeVehicle(port, **kwargs)
        vehicle.start()
        started.append(vehicle)
        return vehicle, f"udpin:127.0.0.1:{port}"

    yield factory
    for vehicle in started:
        vehicle.stop()


@pytest.fixture
def fly(make_vehicle):
    """Connect flypi to a FakeVehicle; returns (drone, vehicle)."""
    drones: list[flypi.Drone] = []

    def factory(*, connect_kwargs=None, **vehicle_kwargs):
        vehicle, address = make_vehicle(**vehicle_kwargs)
        drone = flypi.connect(address, timeout=5, **(connect_kwargs or {}))
        drones.append(drone)
        return drone, vehicle

    yield factory
    for drone in drones:
        drone.close()
