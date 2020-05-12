import QtQuick 2.14
import QtQuick.Window 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14
import QtLocation 5.14
import QtPositioning 5.14
import QtGraphicalEffects 1.14



ApplicationWindow {
    visible: true
    width: 1280
    height: 720
    title: qsTr("Map Planner")
    property MapCircle point
    property MapQuickItem marker
    property bool isPlay : true
    property string playColor: "green"
    property string pauseColor: "red"
    property string pauseText: "Pause"
    property string playText: "Play"

    function setButtonText()
    {
        var buttonText = pauseText

        if(isPlay == true)
        {
            buttonText = playText
        }

        else
        {
            buttonText = pauseText
        }

        return buttonText

    }



    Plugin {
        id: mapPlugin
        name: "mapboxgl"
    }

    MouseArea{
        anchors.fill:parent
        focus:true
        hoverEnabled:false
        acceptedButtons: Qt.LeftButton
    }


    header: ToolBar{
        RowLayout{
            anchors.fill:parent
        }
        background: Rectangle {
            color: "transparent"
        }

        ComboBox {
            id: toolbarComboBox
            Layout.minimumWidth:200
            currentIndex: 0
        }

        Button{
            id:takeoffButton
             anchors{
                left: toolbarComboBox.right
                leftMargin:5
             }
            highlighted:true
            text: "Takeoff"
            onClicked: planner.takeoff()
        }

        Button{
            id:landButton
            anchors{
                left: takeoffButton.right
                leftMargin:5
            }
            text: "Land"
            highlighted:true
            onClicked: planner.land()
        
         }

        Button{
            id:rthButton
            anchors{
                left: landButton.right
                leftMargin:5
            }
            text: "RTH"
            highlighted:true
            onClicked: planner.goHome()
        }

        Button{
            id:pauseButton
            anchors{
                left:rthButton.right
                leftMargin:5
            }   
            highlighted:true     
             text: setButtonText() 
            onClicked:{
                    isPlay = !isPlay
                    planner.getPlayPause = isPlay          
            }
        
        }

        StatusBar{
            id:statBar
            anchors{
                left:pauseButton.right
                leftMargin:5
            }   
        }




    }

    Map{
        id: map
        anchors.fill:parent
        plugin: mapPlugin
        center: QtPositioning.coordinate(52.75591, -1.246310)
        zoomLevel:40
        copyrightsVisible: false
        tilt: 45


        PluginParameter {
                name: "mapboxgl.mapping.cache.memory"
                value: true
        }

        MapItemView{
            model: planner.uavModel
            delegate: MapQuickItem{ // Change to MapQuickItem
                coordinate:model.position
                sourceItem: Image{
                    id:uavImage
                    opacity: .75
                    sourceSize.width:32
                    sourceSize.height:32
                    source: "qrc:///drone.png"
                }
            anchorPoint.x: uavImage.width/4
            anchorPoint.y: uavImage.height/4
            }

           
        }

        Line{
            id: uavPath
        }

        MapItemView{
            model: ListModel {
                id: markerModel
                dynamicRoles: true
             }
            delegate:MapQuickItem{
                coordinate:model.position
                sourceItem: Image{
                    id:waypointMarker
                    opacity: .75
                    sourceSize.width:80
                    sourceSize.height:80
                    source: "qrc:///marker-red.png"
                }
            anchorPoint.x: waypointMarker.width/2
            anchorPoint.y: waypointMarker.height/2
            }
        }

        MouseArea{
        anchors.fill:parent
        focus:true
        hoverEnabled:false
        acceptedButtons: Qt.LeftButton

            onPressAndHold: {
                   var pos = map.toCoordinate(Qt.point(mouse.x,mouse.y));
                   //var markerId = markerModel.count + 1;
                   markerModel.append({"position":pos})
                   uavPath.addCoordinate(pos)

               // missionPopup.open()   
            }
                
        }
                    
   }

    
   

    Waypointparam{
        id: missionPopup
    }

    Button{
        id:abortButton
        anchors {
            bottom: parent.bottom
            right: parent.right
            bottomMargin: 5
            rightMargin: 5
            
        }
        background: Rectangle{
             color: "red"
        }
        text: qsTr("Abort")
        opacity: 1

        onClicked:
        {
            planner.abortMission()
            console.log("abort clicked")
        }
    }

    Button{
        id:startButton
        anchors {
            bottom: parent.bottom
            right: abortButton.left
            bottomMargin: 5
            rightMargin: 5
            
        }
        background: Rectangle{
             color: "green"
        }
       
        text: qsTr("Start")
        opacity: 1

        onClicked:
        {
             
          //  planner.startMission()
            console.log("start clicked")
            
        }
    
    }


}