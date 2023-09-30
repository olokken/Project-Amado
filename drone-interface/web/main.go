package main

import (
	"drones"
	"web/controllers"
	"web/routes"

	"github.com/gin-gonic/gin"
)

func main() {
	router := gin.Default()

	var drone drones.IDrone = &drones.Tello{ConnectionString: "8888"}

	var missionController controllers.IMissionController = &controllers.MissionController{Drone: drone}

	v1 := router.Group("api/v1")
	{
		routes.MissionRoutes(v1, missionController)
	}

	router.Run(":8080")
}
