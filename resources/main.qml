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
    property int missionType

    signal clearMap()


    function createMap()
    {
        if(map)
        {
            map.destroy()
        }

        map = mapComponent.createObject(page);
        map.activeMapType = map.supportedMapTypes[0]
         //map.activeMapType = MapType.SatelliteMapDay

        map.zoomLevel = (map.maximumZoomLevel - map.minimumZoomLevel)/2
        map.tilt = 45
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
    

        onMissionModeChanged:{
            missionType = searchBar.missionMode
             console.debug("Mission Mode:" + missionMode)

        }
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
                    property int missionMode : missionType  
          
                    onErrorChanged: {
                        if(map.error !== Map.NoError){
                        var title = qsTr("ProviderError");
                        var message = map.errorString + "<br/>";
                        if(map.error === Map.MissingRequiredParameterError)
                        message += "Missing Paramater";
                        console.log(Title + " :" + message);
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
         }    
    }

    Button{
    id:resetButton
    anchors{
        right:parent.right
        bottom:parent.bottom
        rightMargin: 5
        bottomMargin: 10
    }
    text:"Reset"
    onClicked:{
        planner.reset()
        clearMap()
    }
}


}
