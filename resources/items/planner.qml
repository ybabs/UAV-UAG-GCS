import QtQuick 2.14
import QtQuick.Window 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14
import QtLocation 5.14
import QtPositioning 5.14
import QtGraphicalEffects 1.14



ApplicationWindow {
    visible: true
    width: 1024
    height: 768
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
        name:  "osm" //"esri"//"osm"//"mapboxgl"

        // PluginParameter {
        //             name: "mapboxgl.mapping.use_fbo"
        //             value: true
        // }

        // PluginParameter {
        //         name: "mapboxgl.mapping.items.insert_before"
        //         value: "aerialway"
        // }
    }
    //     Plugin {
    //         id: mapPlugin
    //         name: "here"
    //         PluginParameter { name: "here.app_id"; value: "3zdF7KMfeuvuAcPeqbxU" }
    //         PluginParameter { name: "here.token"; value: "QYTloDJZoR1sz7W1nRs2Dz9iGwLUDC9WVzVjUMqj7Ww" }
    //     }
    
        
 

    MouseArea{
        anchors.fill:parent
        focus:true
        hoverEnabled:false
        acceptedButtons: Qt.LeftButton
    }
    ListModel {
        id:wpModel
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
             model:planner.uavModel
             textRole: "identifier"
            onActivated:
            {
                var value = delegateModel.items.get(currentIndex)
                console.log(value.model.identifier);
               
                
            }
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
                    planner.missionStatusFlag = isPlay          
            }
        
        }

        StatusBar{
            id:statBar
            anchors{
                left:pauseButton.right
                leftMargin:5
            }   
        }

        Button{
            id:waypointButton
            anchors{
                right: parent.right
            }
            highlighted:true
            text: "Set Waypoints"

            Popup {
                id: wpinputpopup
                 parent: Overlay.overlay
                x: Math.round((parent.width - width) / 2)
                y: Math.round((parent.height - height) / 2)
                width: 600
                height: 200
                dim:true
                visible:false

                Overlay.modal:Rectangle {
                     color: "#aacfdbe7"
                }

                ColumnLayout{
                anchors{
                    right:parent.right
                    left:parent.left
                }
                spacing:10
                RowLayout{
                    Label{
                        text: "Start Lat:"
                    } 
                    TextField{
                        id: startLatInput
                        validator:DoubleValidator{
                            decimals:17
                            notation:DoubleValidator.ScientificNotation
                        }
                    }
                    Label{
                        text:"Start Lon:"
                    }
                    TextField{
                        id: startLonInput
                        validator:DoubleValidator{
                            decimals:17
                            notation:DoubleValidator.ScientificNotation
                        }
                    }  
                } //RowLayout
                RowLayout{
                    Label{
                        text: "End Lat:"
                    } 
                    TextField{
                        id: endLatInput
                        validator:DoubleValidator{
                            decimals:17
                            notation:DoubleValidator.ScientificNotation
                        }
                    }
                    Label{
                        text:"End Lon:"
                    }
                    TextField{
                        id: endLonInput
                        validator:DoubleValidator{
                            decimals:17
                            notation:DoubleValidator.ScientificNotation
                        }
                    }

  
                } //RowLayout
                GroupBox{
                    RowLayout{
                        Button{
                            id: wpOkButton
                            text: "OK"
                            onClicked:{
                                var startLat = parseFloat(startLatInput.text)
                                var startLon = parseFloat(startLonInput.text)
                                var val = map.toCoordinate(startLat, startLon) //conversion
                                wpModel.append({lat: val.latitude, lon: val.longitude})
                                 //wpModel.append({lat: 52.755949, lon: -1.247106})
                                 console.log(val.latitude)
                                 console.log(val.longitude)
                                console.log(startLonInput.text) 
                                console.log(startLatInput.text)   
                                // console.log(val.latitude - startLatInput.text)
                                // console.log(val.longitude - startLonInput.text)
                                point = Qt.createQmlObject('import QtLocation 5.14;\
                                                    MapCircle {radius: 0.5; color: "red"; opacity: 0.5; border.width: 0.5}', map)
                                point.center = QtPositioning.coordinate(startLat, startLon)
                                map.addMapItem(point)
                                                          
                                wpinputpopup.close()
                            }
                        } // okButton

                        Button{
                            id:wpCancelButton
                            text: "Cancel"
                            onClicked: {
                                wpinputpopup.close()
                            }
                        } // CancelButton

                    } // RowLayout
                } // GroupBox
                }// ColumnLayout
            } // popup

            
            onClicked:{

                wpinputpopup.open()

            }
        } // Button
    }  

    Map{
        id: map
        anchors.fill:parent
        plugin: mapPlugin
        center: QtPositioning.coordinate(52.755949, -1.247106)
        zoomLevel:18
        copyrightsVisible: false
        tilt: 45
         //activeMapType: supportedMapTypes[4]
        activeMapType: supportedMapTypes[1]

            ComboBox {
                Layout.minimumWidth: 200

                currentIndex: 0

                model: ListModel {
                    id: styleModel
                }

                onActivated: {
                    map.activeMapType = map.supportedMapTypes[index];
                }
            }


        PluginParameter {
                name: "mapboxgl.mapping.cache.memory"
                value: true
        }

         MapCircle {
             id:sample_circle
        center {
            latitude: 52.756031
            longitude: -1.247403
        }
        radius: 1
        color: 'green'
        border.width: 3
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
            anchorPoint.y: uavImage.height
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

        MapItemView{
            model:wpModel
            delegate:MapQuickItem{
            coordinate: QtPositioning.coordinate(lat,lon)
                sourceItem: Image{
                    id:waypointMarker
                    opacity: .75
                    sourceSize.width:80
                    sourceSize.height:80
                    source: "qrc:///marker-red.png"
                }
            
            // anchorPoint.x: waypointMarker.width/2
            // anchorPoint.y: waypointMarker.height

            }
        }

        MapQuickItem{
            sourceItem: Image{
                id:sample
                opacity: .75
                sourceSize.width:80
                sourceSize.height:80
                source: "qrc:///marker-red.png"
            }
            coordinate: QtPositioning.coordinate(48.854314, 2.292297)
            anchorPoint.x: sample.width/20
            anchorPoint.y: sample.height

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

                   console.log(pos.latitude)
                   console.log(pos.longitude)

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