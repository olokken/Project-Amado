package mission

import (
	"github.com/gin-gonic/gin"
)

var prefix string = "mission"

func MissionRoutes(router *gin.RouterGroup, controller IMissionController) {
	router.POST(prefix, controller.PlanMission)
}
