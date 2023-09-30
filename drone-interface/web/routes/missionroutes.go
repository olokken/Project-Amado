package routes

import (
	"web/controllers"

	"github.com/gin-gonic/gin"
)

var prefix string = "mission"

func MissionRoutes(router *gin.RouterGroup, controller controllers.IMissionController) {
	router.POST(prefix, controller.PlanMission)
}
