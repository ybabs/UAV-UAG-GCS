import QtQuick 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14
import QtQuick.Controls.Material 2.14

RowLayout{

    signal tspwaypointGenerated(ListModel tspwaypoints)


    ListModel{
        id: tspList
    }


    anchors{
        bottom: parent.bottom
        left: parent.left
        bottomMargin: 15
//        leftMargin: 5
    }

    Button {
        id:abortButton
        text: "Abort"
        opacity: 1
        Material.accent: Material.Red
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
        Material.accent: Material.Green
        onClicked: {

            planner.startMission()

            for(var i = 0; i < planner.tspPath.length; i++)
            {
                
                tspList.append(planner.tspPath[i])
                //console.log(planner.tspPath[i].latitude)
            }

             tspwaypointGenerated(tspList) 
            console.log("Starting Mission ")
        }

    }

}
