package main

import (
	"github.com/gin-gonic/gin"
	"github.com/olokken/Project-Amado/droneInterface/internal/drones"
	"github.com/olokken/Project-Amado/droneInterface/internal/web/mission"
)

func main() {
	router := gin.Default()

	var drone drones.IDrone = &drones.Tello{ConnectionString: "8888"}

	var missionController mission.IMissionController = &mission.MissionController{Drone: drone}

	v1 := router.Group("api/v1")
	{
		mission.MissionRoutes(v1, missionController)
	}

	router.Run(":8080")
}
