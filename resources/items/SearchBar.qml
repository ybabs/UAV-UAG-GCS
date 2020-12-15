import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15


ToolBar{

    property bool busyIndicatorRunning: false
    property bool searchBarVisible: true
    property bool inPlay: true
    property int missionMode
    property string pauseText: "Pause"
    property string playText: "Play"

    signal showMap()
    signal missionTypeChanged(int value)

    onSearchBarVisibleChanged: {
        searchBar.opacity = searchBarVisible ? 1: 0
    }
    
    function setButtonText()
    {
        var buttonText = pauseText
        if(inPlay === true)
        {
            buttonText = pauseText
        }

        else{
            buttonText = playText
        }

        return buttonText
    }

    RowLayout{
        id:searchBar
        width:parent.width
        height: parent.height
        ComboBox {
            Layout.minimumWidth: 200
            id:modelCombo
            currentIndex:0
            model: ListModel {
                id: missionModeModel
                ListElement {text: "Single Mode"}
                ListElement{text: "Swarm Mode"}
                ListElement{text: "Transect Mode"}
            }

            onCurrentIndexChanged:{
                missionMode = currentIndex
                missionTypeChanged(missionMode)
            }
        }

        Button{
            id:takeoffButton
            text:"Takeoff"
            onClicked: planner.takeoff()
        }

        Button{
            id:landButton
            text:"Land"
            onClicked: planner.land()
        } 

        Button{
            id:rthButton
            text:"RTH"
            onClicked:planner.goHome()
        }

        Button{
            id:pauseButton
            text:setButtonText()
            onClicked:{
                inPlay = !inPlay
                planner.missionStatusFlag = inPlay
            }
        }

 

        MavComponent{

        }

        Button{
            id: setMavControlButton
            highlighted:true
            text: "Upload"
            onClicked: {                
                planner.uploadWaypoints()
            }

        }
 




    }

}
