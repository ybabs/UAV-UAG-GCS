Rectangle{
    id: popUpContainer
    property alias popUpMessage: popUpMessage.text
    PropertyAnimation { id: showPopUp; target: popUp; property: "opacity"; to: 1; duration: 500; easing.type: Easing.Linear}
   PropertyAnimation { id: hidePopUp; target: popUp; property: "opacity"; to: 0; duration: 500; easing.type: Easing.Linear}
	color: "transparent"

    Rectangle{
        id: popUp
		
        Text{
            id: popUpMessage
        }
    }
}