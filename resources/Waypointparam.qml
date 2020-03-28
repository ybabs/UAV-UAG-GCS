
import QtQuick 2.14
import QtQuick.Window 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14

    Popup
    {
        x: (parent.width - width) / 2
        y: (parent.height - height) / 2
      
        height:250
        modal:true
        focus:true
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutsideParent

        contentItem :GridLayout{
            width:parent.width
            columns:1
            columnSpacing:2
            
             Label{
                id:altitudeLabel
                text: "Altitude (< 100m)" 
                font.bold:false
                Layout.alignment: Qt.AlignTop | Qt.AlignVCenter             
            }

           TextField{
                id:altitudetextInput
                Layout.alignment: Qt.AlignTop 
                 validator: IntValidator{
                     bottom: 1
                     top: 100
                 }
                 Layout.fillWidth:false
                 width: parent.width * 0.5
                 background: Rectangle {
                    color:"#7C807D"
                    radius: 10
                    opacity: 0.2
                    width: 50

                 }   
                 focus:true
                 font.bold:true  

                onEditingFinished: {
                     planner.getAltitude= altitudetextInput.text
                }          

            }


            CheckBox {
                 id:landCheckBox
                 text:"Land and Record"
                 leftPadding: 0
                  Layout.alignment: Qt.AlignLeft 
                 onClicked:
                 {
                     if(checked ){
                         planner.getSamplingFlag = 1
                     }

                     else{
                         planner.getSamplingFlag = 0
                     }
                 }                 
             }

             

             Label{
                id:samplingLabel
                text: "Sampling Time"
                Layout.alignment: Qt.AlignTop 
                font.bold:false
             }

            TextField{
                 id:sampletextField
                  Layout.alignment: Qt.AlignTop 
                 validator: IntValidator{
                     bottom: 1
                     top: 800
                 }
                  background: Rectangle {
                    color:"#7C807D"
                    radius: 10
                    opacity: 0.2
                    width: 50

                 }   
                 Layout.fillWidth:false
                 focus:true
                 font.bold:true   

                onEditingFinished:{
                     planner.getSamplingTime= sampletextField.text
                }     

                inputMethodHints: Qt.ImhDigitsOnly      
             } 

             GroupBox{
                 RowLayout{
                    anchors.fill:parent
                    Button{
                        text: "OK"
                        id:okPopupbutton
                        Layout.alignment: Qt.AlignLeft
                        onClicked:{
                            planner.addWaypoint(point.center.latitude, point.center.longitude, 
                                                planner.getAltitude, planner.getSamplingFlag,
                                                planner.getSamplingTime)
                            missionPopup.close()
                        }               
                    }
              
                    Button{
                        id:cancelPopupButton
                        text: "Cancel"
                        Layout.alignment:  Qt.AlignRight 
                        onClicked:{
                             missionPopup.close()
                        }
                    }
                 }
             }

        }                  
    }