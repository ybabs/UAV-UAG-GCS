import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Popup
{
    x: (parent.width - width) / 2
    y: (parent.height - height) / 2
    height: 310
    visible: false
    focus:true

    signal waypointGenerated(ListModel waypoints)


    ListModel{
        id: pointsList
    }

    ColumnLayout{

        TextField{
            id: centerSearchText
            Behavior on opacity {NumberAnimation{}}
            property bool ignoreTextChange: false
            placeholderText: qsTr("Enter center coordinate")
            Layout.fillWidth:true
            text: "52.772040, -1.208703"
 
        }

        TextField{
            id: centerDistanceText
            Behavior on opacity {NumberAnimation{}}
            placeholderText: "Enter Distance (m)"
            Layout.fillWidth:true
            validator: IntValidator{bottom: 1; top: 1000}
            inputMethodHints: Qt.ImhDigitsOnly
            text:"300"
         }

         TextField{
             id: samplingTimeText
             validator: IntValidator{
                 bottom: 1
                 top: 800
             }
             Layout.fillWidth:true
             placeholderText: "Enter Sampling Time"
             text:"10"
             focus:true
            inputMethodHints: Qt.ImhDigitsOnly
         }
         TextField{
             id:droneSpeedText
             validator: IntValidator{
                 bottom: 1
                 top: 15
             }
             Layout.fillWidth:true
             placeholderText: "Enter Speed"
             focus:true
             text:"5"
             onEditingFinished: planner.droneSpeed = droneSpeedText.text
            inputMethodHints: Qt.ImhDigitsOnly
         }

         TextField{
             id:hydrophoneRadius
             validator: IntValidator{
                 bottom:1
                 top: 300
             }
             Layout.fillWidth:true
             placeholderText: "Hydrophone radius"
             focus:true
             text:"38"
             onEditingFinished: planner.hydrophoneRange = hydrophoneRadius.text
             inputMethodHints:Qt.ImhDigitsOnly

         }

         GroupBox{
             RowLayout{
                 Button{
                     id:okButton
                     text: "OK"
                     onClicked:{
                         planner.generateDisks(centerSearchText.text, centerDistanceText.text, samplingTimeText.text)

                         for (var i in planner.trackpoints){
                             var p = planner.trackpoints[i];
                             pointsList.append(p)
                         }
                         waypointGenerated(pointsList) 
                         diskGenPopup.close();
                     }

                 }

                 Button{
                     id:cancelButton
                     text:"Cancel"
                     onClicked:{
                         diskGenPopup.close()
                     }
                 }
             }
         }
    }
    
}