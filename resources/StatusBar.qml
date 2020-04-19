import QtQuick 2.14
import QtQuick.Window 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14

Rectangle{
    width:260
    height:80
    color: "#7C807D"
    radius:10
    opacity: 0.1

    anchors{
        bottomMargin:5
        bottom: parent.bottom
        left: parent.left
        leftMargin:5
    }
 
    ListView{
        anchors.fill:parent
        anchors.margins: 20
        spacing: 4
        clip:true
        model:planner.uavModel
        orientation:ListView.Horizontal
        delegate:batteryDelegate
        focus: true
    }

    Component{
        id:batteryDelegate
        GreenBox {
             width: 50
             height: 50
             text:model.battery
             
        }
    }

}
