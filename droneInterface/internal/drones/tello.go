package drones

import (
	"time"

	"github.com/olokken/Project-Amado/droneInterface/internal/domain"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
)

type Tello struct {
	ConnectionString string
	Driver           *tello.Driver
}

func NewTelloDrone(connectionstring string) *Tello {
	return &Tello{
		ConnectionString: connectionstring,
		Driver:           tello.NewDriver(connectionstring),
	}
}

func (t *Tello) HandleMission(mission domain.Mission) {
	work := func() {
		t.Driver.On(tello.ConnectedEvent, func(data interface{}) {
			t.Driver.TakeOff()
			t.Driver.Forward(50)

			gobot.After(2*time.Second, func() {
				t.Driver.Land()
			})
		})
	}

	robot := gobot.NewRobot("tello",
		[]gobot.Connection{},
		[]gobot.Device{t.Driver},
		work,
	)

	robot.Start(false)
}

func (t *Tello) Forward(length float64) {
	t.Driver.Forward(int(length))
}

func (t *Tello) ClockwiseTurn(angle float64) {
	t.Driver.Clockwise(int(angle))
}

func (t *Tello) CounterclockwiseTurn(angle float64) {
	t.Driver.CounterClockwise(int(angle))
}
