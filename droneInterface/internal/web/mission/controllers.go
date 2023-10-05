package mission

import (
	"github.com/olokken/Project-Amado/droneInterface/internal/domain"
	"github.com/olokken/Project-Amado/droneInterface/internal/drones"

	"github.com/gin-gonic/gin"
)

type IMissionController interface {
	HandleMission(c *gin.Context)
}

type MissionController struct {
	Drone drones.IDrone
}

func (m *MissionController) HandleMission(c *gin.Context) {
	var mission domain.Mission

	if err := c.BindJSON(&mission); err != nil {
		c.JSON(400, gin.H{
			"error": "Invalid request data",
		})
		return
	}

	m.Drone.HandleMission(mission)
}
