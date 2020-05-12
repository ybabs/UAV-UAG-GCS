import QtQuick 2.14
import QtQuick.Window 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14

Rectangle{
    width:260
    height:40
    color: "transparent"
    radius:10 


    Text{
        id: batteryText
        text: '<b>Battery:</b>'
        anchors.verticalCenter: parent.verticalCenter
        
    }
 
    ListView{
        // anchors.fill:parent
        anchors{
            left: batteryText.right
            leftMargin: 5
        }
        width: parent.width
        height:parent.height
        spacing: 4
        model:planner.uavModel
        orientation:ListView.Horizontal
        delegate:batteryDelegate
        focus: true
    }

    Component{
        id:batteryDelegate
        GreenBox {
            height:40
            width:40
            color:"transparent"
             Text{
                 text:model.battery
                 anchors.verticalCenter : parent.verticalCenter
                 font.pixelSize: 12

             }
             
             
        }
    }

}
