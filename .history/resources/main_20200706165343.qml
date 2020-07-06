import QtQuick 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14
import QtPositioning 5.14
import QtLocation 5.14
import "items"

ApplicationWindow{
    id:appWindow
    property Map map
    property MapItemView waypointItemView
    property variant searchLocation: map ? map.center: QtPositioning.coordinate()
    property variant searchRegion: QtPositioning.circle(searchLocation)
    property variant searchRegionItem

  ListModel{
      id:waypointModel
  }

    function createMap()
    {
        if(map)
        {
            map.destroy()
        }

        map = mapComponent.createObject(page);
        map.activeMapType = map.supportedMapTypes[1]
        map.zoomLevel = (map.maximumZoomLevel - map.minimumZoomLevel)/2
        map.tilt = 45
    }

    function clearMap()
    {
        map.clearMapItems();
        map.clearData();
        waypointModel.clear();
    }

    function initializeProviders()
    {
        createMap()
    }

    title: qsTr("Sample")
    width: 1280
    height: 720
    visible: true

    header: SearchBar{
        id:searchBar
        width:appWindow.width
        searchBarVisible: stackView.depth > 1 &&
                          stackView.currentItem &&
                          stackView.currentItem.objectName !== "suggestionView" ? false : true

        onWaypointGenerated:{  
            waypointModel.clear()               
            for(var i = 0; i < waypoints.count; i++){
                waypointModel.append(waypoints.get(i))
            }    
            waypoints.clear()           
            console.log(waypoints.count)
            console.log(waypointModel.count)
        }       

        onSwarmModeChecked: {
            MapComponent.missionType = searchBar.swarmCheckState
         }
        onShowMap: stackView.pop(page)
    }


    StackView{
        id:stackView
        anchors.fill: parent
        focus:true
        initialItem: Item {
            id: page

            Component{
                id: mapComponent

                MapComponent{
                    width:page.width
                    height: page.height

                        onErrorChanged: {
                            if(map.error !== Map.NoError){
                                var title = qsTr("ProviderError");
                                var message = map.errorString + "<br/>";
                                if(map.error === Map.MissingRequiredParameterError)
                                    message += "Missing Paramater";
                                console.log(Title + " :" + message);
                            }
                        }

                        MapItemView{
                            id:waypointItemView
                            model:waypointModel
                            delegate: MapQuickItem{
                                coordinate: QtPositioning.coordinate(model.latitude, model.longitude)
                                anchorPoint.x: image.width * 0.28
                                anchorPoint.y: image.height


                                sourceItem: Image {
                                    id:image
                                    source: "../images/marker.png"
                                }
                            }


                        }

                        

                }

            }


        }
    }

    ControlComponent{
        id: controlButtons

    }

    StatusBar{
        id:statBar
          anchors{
              left:controlButtons.right
              bottom: controlButtons.bottom
              leftMargin:5
            //   bottomMargin:10
         }    


    }


}
