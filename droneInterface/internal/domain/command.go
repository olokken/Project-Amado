package domain

type Instruction struct {
	// metric can be in length (meters) or angle (degrees)
	Metric  float64
	Command Command
}

type Command int

const (
	Forward Command = iota
	Backward
	Left
	Right
	ClockwiseTurn
	CounterclockwiseTurn
)
