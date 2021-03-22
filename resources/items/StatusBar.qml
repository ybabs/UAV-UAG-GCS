import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

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
    }

    Component{
        id:batteryDelegate
        Rectangle {
            id:statusRect
            height:40
            width:40
            Text{
                 text:model.battery
                 anchors.verticalCenter: parent.verticalCenter 
                 anchors.left: parent.left
                 font.pixelSize: 15
                 onTextChanged: {
                     if(model.battery > 50) {statusRect.color = "#53d769"; return;}
                     if(model.battery < 20) {statusRect.color = "#ff0000"; return;}  
                     if(model.battery > 20 && model.battery < 49) {statusRect.color="#FFBF00"; return;}
                 }
             }
             
             
        }
    }

}
