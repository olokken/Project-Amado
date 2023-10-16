package drones

import (
	"log"
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
var direction string

func (t *Tello) HandleCommand(instructions []domain.Instruction) {
	log.Println(instructions)
	if len(instructions) == 0 {
        return
    }
    instruction := instructions[0]

	switch instruction.Command {
    case 0:
		t.Driver.Forward(int(instruction.Metric))
		log.Println("Going forward")
		direction = "forward"
		gobot.After(5*time.Second, func() {
			t.Driver.Hover()
			direction = ""
			log.Println("Stop froward")
		})
		
    case 1:
		t.Driver.Backward(int(instruction.Metric))
		log.Println("Going forward backward")
		gobot.After(6*time.Second, func() {
			t.Driver.Backward(int(0))
			log.Println("Stop froward")
		})
	case 2:
		t.Driver.Left(int(instruction.Metric))
		direction = "left"
		gobot.After(6*time.Second, func() {
			direction = ""
			t.Driver.Left(int(0))
		})
	case 3:
		t.Driver.Right(int(instruction.Metric))
		gobot.After(6*time.Second, func() {
			t.Driver.Right(int(0))
		})
	case 4:
		t.Driver.Clockwise(int(instruction.Metric))
		log.Println("Going clockwise")
		gobot.After(5*time.Second, func() {
			t.Driver.Hover()
			log.Println("Stop clockwise")
		})
	case 5:
		t.Driver.CounterClockwise(int(instruction.Metric))
		log.Println("Going CounterClockwise")
		gobot.After(3*time.Second, func() {
			t.Driver.CounterClockwise(int(0))
			log.Println("Stop CounterClockwise")
		})
  
    default:
        log.Println("Unknown command")
    }
	
	//Put time variable in instruction and figure out ho wwe do it with commands
	gobot.After(5*time.Second, func() {
    t.HandleCommand(instructions[1:])
})

}

func (t *Tello) HandleMission(mission domain.Mission) {
	
	work := func() {
		t.BatteryStatus()
		
	 t.Driver.On(tello.ConnectedEvent, func(data interface{}) {
		log.Println(data)

		t.Driver.TakeOff()
		t.HandleCommand(mission.Instructions)
		log.Println(mission)
		 
		gobot.After(5*time.Second, func() {
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


func (t *Tello) BatteryStatus() {
		var flightData *tello.FlightData
		
		t.Driver.On(tello.FlightDataEvent, func(data interface{}) {
			flightData = data.(*tello.FlightData)
			log.Println(flightData.EastSpeed)
			if flightData.EastSpeed > 0 && direction != "right" {
				log.Println("hei")
				t.Driver.Left(20)
				gobot.After(1*time.Second, func() {
					t.Driver.Right(int(0))
				})
			}
			/* gobot.Every(2*time.Second, func() {
				if flightData.BatteryPercentage > 10 {
					log.Printf("The drone has: %d battery.\n", flightData)
				}  else  {
					log.Println("Too low battery", flightData, "Shutting down")
					t.Driver.Halt()
					return
				}
			}) */
			
		})

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
