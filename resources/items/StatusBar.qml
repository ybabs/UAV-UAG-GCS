import QtQuick 2.14
import QtQuick.Window 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14

Rectangle{
    width:260
    height:40
    color: "#AEA79F"
    radius:10 


    Text{
        id: batteryText
        text: '<b>Battery:</b>'
        anchors.verticalCenter: parent.verticalCenter
        
    }
 
    ListView{
        anchors{
            left: batteryText.right
            leftMargin: 5
        }
        width: parent.width
        height:parent.height
        spacing: 4
        model:mav.uavModel
        orientation:ListView.Horizontal
        delegate:batteryDelegate
        focus: true
    }

    Component{
        id:batteryDelegate
        Box {
            height:40
            width:40
             Text{
                 text:model.battery
                anchors.verticalCenter: parent.verticalCenter 
                anchors.left: parent.left
                 font.pixelSize: 15

             }
             
             
        }
    }

}
