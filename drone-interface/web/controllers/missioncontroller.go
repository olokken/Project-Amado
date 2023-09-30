package controllers

import (
	"drones"

	"github.com/gin-gonic/gin"
)

type IMissionController interface {
	PlanMission(c *gin.Context)
}

type MissionController struct {
	Drone drones.IDrone
}

func (m *MissionController) PlanMission(c *gin.Context) {
	m.Drone.PlanMission()
}
