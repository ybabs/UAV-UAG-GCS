import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Controls.Material 2.15


// Contains the control buttons to start and abort 
// missions. startButton also generates a signal when 
// tsp waypoints are generated in single mode. 

RowLayout{

    signal tspwaypointGenerated(ListModel tspwaypoints)

    ListModel{
        id: tspList
    }

    anchors{
        bottom: parent.bottom
        left: parent.left
        bottomMargin: 15
        leftMargin: 5
    }

    Button {
        id:abortButton
        text: "Abort"
        opacity: 1
        palette{
            button: "#f55d42"
        }
        onClicked: {
            planner.abortMission()
            console.log("Aborting Mission")
            for(var i = 0; i < planner.mtspPath.length; i++)
            {
                 console.log(planner.mtspPath[i])

                for(var j = 0; j < planner.mtspPath[i].length; j++)
                {
                    console.log(planner.mtspPath[i][j].latitude)
                }
            }
        }

    }

    Button{
        id:startButton
        text:"Start"
        palette{
            button: "#45f542"
        }
        onClicked: {
            planner.startMission()
            for(var i = 0; i < planner.tspPath.length; i++)
            {
                tspList.append(planner.tspPath[i])
                console.log(planner.tspPath[i].latitude)
            }
            tspwaypointGenerated(tspList) 
            console.log("Starting Mission ")
        }

    }

}
