import QtQuick 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14
import QtPositioning 5.14
import QtLocation 5.14
import "items"

ApplicationWindow{
    id:appWindow
    property Map map
    property variant parameters
    property variant searchLocation: map ? map.center: QtPositioning.coordinate()
    property variant searchRegion: QtPositioning.circle(searchLocation)
    property variant searchRegionItem


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
        placeSearchModel.plugin = map.plugin;

    }

    function splitString(text)
    {
        var lat;
        var lon;
        var stringList;

        stringList = text.split(',');
        lat = stringList[0];
        lon = stringList[1];

        //console.log("Lat:" + lat);
        //console.log("Lon: " + lon);


        map.center = QtPositioning.coordinate(parseFloat(lat), parseFloat(lon));
        console.log(parseFloat(lon));

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
        onStartSearch: {
            if(searchCoord.length > 0){
                placeSearchModel.searchForText(searchCoord);
                splitString(searchCoord);

            }
        }

        onShowMap: stackView.pop(page)
    }



    StackView{
        id:stackView
        anchors.fill: parent
        focus:true
        initialItem: Item {
            id: page

            PlaceSearchModel{
                id: placeSearchModel
                searchArea: searchRegion

                function searchForText(text){
                    searchTerm = text;
                    categories = null;
                    recommendationId = "";
                    searchArea = searchRegion
                    limit = -1;
                    update();
                }
                // onStatusChanged: {
                //     switch (status){

                //     case placeSearchModel.Ready:
                //         if(count > 0)
                //         {
                //             console.log("Coordinate ready");
                //         }
                //         else
                //             console.log("Search Error");
                //         break;
                //     case placeSearchModel.Error:
                //         console.log("Search Place error")
                //         break;
                //     }
                // }

            }

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
                        model:placeSearchModel
                        delegate: MapQuickItem{
                            //coordinate: model.type === PlaceSearchModel.PlaceResult  ? place.location.coordinate : QtPositioning.coordinate()
                            //visible: model.type === PlaceSearchModel.PlaceResult
                            coordinate:place.location.coordinate
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

    Rectangle {
        color: "white"
        opacity: busyIndicator.running ? 0.8 : 0
        anchors.fill: parent
        Behavior on opacity { NumberAnimation{} }
    }

    BusyIndicator{
        id: busyIndicator
        anchors.centerIn: parent
        running: placeSearchModel.status === PlaceSearchModel.Loading
    }



    ControlComponent{

    }




}
