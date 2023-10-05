package drones

import (
	"github.com/olokken/Project-Amado/droneInterface/internal/domain"
)

type IDrone interface {
	HandleMission(mission domain.Mission)
	Forward(length float64)
	ClockwiseTurn(angle float64)
	CounterclockwiseTurn(angle float64)
}

func SetupMission(drone IDrone, mission domain.Mission) {
	for _, value := range mission.Instructions {
		if value.Command == domain.Forward {
			drone.Forward(value.Metric)
		} else if value.Command == domain.ClockwiseTurn {
			drone.ClockwiseTurn(value.Metric)
		} else if value.Command == domain.CounterclockwiseTurn {
			drone.CounterclockwiseTurn(value.Metric)
		}
	}
}
