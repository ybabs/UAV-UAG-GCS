import QtQuick 2.14
import QtQuick.Window 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14

Popup
{
    x: (parent.width - width) / 2
    y: (parent.height - height) / 2
    height: 230
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
            // onTextChanged:{
            //     if(!ignoreTextChange){
            //         searchTextChanged(text)
            //     }
            // }
            // onAccepted: startSearch(text) 
        }

        TextField{
            id: centerDistanceText
            Behavior on opacity {NumberAnimation{}}
            placeholderText: "Enter Distance (m)"
            Layout.fillWidth:true
            validator: IntValidator{bottom: 1; top: 1000}
            inputMethodHints: Qt.ImhDigitsOnly
         }

         TextField{
             id: samplingTimeText
             validator: IntValidator{
                 bottom: 1
                 top: 800
             }
             Layout.fillWidth:true
             placeholderText: "Enter Sampling Time"
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
             onEditingFinished: planner.droneSpeed = droneSpeedText.text
            inputMethodHints: Qt.ImhDigitsOnly
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