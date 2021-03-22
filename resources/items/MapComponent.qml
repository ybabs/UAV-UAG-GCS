import QtQuick 2.15
import QtQuick.Controls 2.15
import QtPositioning 5.15
import QtLocation 5.15
import QtGraphicalEffects 1.15
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15

import "../utils/formatDist.js" as Helper

Map{
    property MapCircle waypoint 
    id:map
    anchors.fill: parent
    property bool followme: true
    property var pos
    property variant scaleLengths: [5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000, 1000000, 2000000]
    
    ListModel{
        id:disksModel
    } 

    ListModel{
        id:boundingBoxModel
    }

    ListModel{
    id:waypointModel
    }     

    ListModel{
        id:markerModel
    }  
    
    function computeScale()
    {
        var coord1, coord2, dist, text, f
        f = 0
        coord1 = map.toCoordinate(Qt.point(0, scale.y))
        coord2 = map.toCoordinate(Qt.point(0+scaleImage.sourceSize.width, scale.y))
        dist = Math.round(coord1.distanceTo(coord2))

        if(dist === 0){
            // not visible
        }
        else {

            for(var i = 0; i < scaleLengths.length-1; i++){
                if(dist< (scaleLengths[i] + scaleLengths[i+1] / 2)) {
                    f = scaleLengths[i]/dist
                    dist = scaleLengths[i]
                    break;
                }
            }
            if(f === 0){
                f = dist / scaleLengths[i]
                dist = scaleLengths[i]
            }

        }
        text = Helper.formatDistance(dist)
        scaleImage.width = (scaleImage.sourceSize.width * f) - 2 * scaleImageLeft.sourceSize.width
        scaleText.text = text
    }

    // plugin:Plugin{
    //     name:"mapboxgl"
    //     PluginParameter {
    //         name: "mapboxgl.mapping.use_fbo"
    //         value: true
    //     }

    //     PluginParameter {
    //         name: "mapboxgl.mapping.items.insert_before"
    //         value: "aerialway"
    //     }

    //     PluginParameter {
    //         name: "mapboxgl.mapping.cache.directory";
    //         value: "~/esri_cache"
    //     }
    //     PluginParameter{
    //         name:"mapboxgl.access_token"
    //         value:"sk.eyJ1IjoibWFza2VkYWZyaWNhbiIsImEiOiJja2lxNWg3eWUwY2lpMnlyeG53bnBmdDVwIn0.-5LFYYl2fmGwEsrGr8g3nA"
    //     }

    //     PluginParameter{
    //         name: "mapbox.mapping.additional_map_ids"; value: "mapbox.mapbox-streets-v8"
    //     }

    //      PluginParameter{
    //         name: "mapboxgl.api_base_url"; value: "https://api.mapbox.com"
    //     }
        
    //  }

    //     plugin:Plugin{
    //     name: "mapbox"//tested 7.16.2020
    //     PluginParameter { //scribe token active(not used)
    //         name: "mapbox.access_token"; value: "pk.eyJ1IjoiamNvZmZpZWxkIiwiYSI6ImNrY3RtYjdoaTBpajcycnFscW1lZWFudHoifQ.TQjmanqldNCYI7P4dOJ4eg"
    //     }
    //     PluginParameter{
    //         name: "mapbox.mapping.additional_map_ids"; value: "mapbox.mapbox-streets-v8"
    //     }

    // }


    plugin:Plugin{
    name:"esri"
    


    }

    // plugin:Plugin {
    //     name: "here"
    //     PluginParameter { name: "here.app_id"; value: "WEdwl6NZWuCYL0ruUgIw" }
    //     PluginParameter { name: "here.token"; value: "eyJhbGciOiJSUzUxMiIsImN0eSI6IkpXVCIsImlzcyI6IkhFUkUiLCJhaWQiOiJHOEMwR2YzZmxZajVybkswZFlGNyIsImlhdCI6MTU5MzM5NzUzMywiZXhwIjoxNTkzNDgzOTMzLCJraWQiOiJqMSJ9" }
    //     PluginParameter { name: "here.proxy"; value: "system" }

    // }



    center {
        latitude:52.769862
        longitude: -1.272527
    }
    gesture.flickDeceleration: 3000
    gesture.enabled: true

    onCenterChanged: {
        scaleTimer.restart()
        if(map.followme)
        {
            // if(map.center !== positionSource.position.coordinate)
            // {
            //     map.followme = false
            // }
        }

    }
    onZoomLevelChanged: {
        scaleTimer.restart()
        if(map.followme)
        {
            //map.center = positionSource.position.coordinate
        }
    }

    onWidthChanged: {
        scaleTimer.restart()
    }

    onHeightChanged: {
        scaleTimer.restart()
    }

    Timer{
        id:scaleTimer
        interval:100
        running: false
        repeat: false
        onTriggered: {
            map.computeScale()
        }
    }

    Item {
        id:scale
        visible: scaleText.text !== "0 m"
        anchors{
            bottom:parent.bottom
            right: parent.right
            margins: 20
        }
        height: scaleText.height * 2
        width: scaleImage.width

        Image {
            id: scaleImageLeft
            source: "../images/scale_end.png"
            anchors{
                bottom: parent.bottom
                right: scaleImage.left
            }
        }

        Image{
            id:scaleImage
            source:"../images/scale.png"
            anchors{
                bottom:parent.bottom
                right:scaleImageRight.left
            }
        }

        Image {
            id: scaleImageRight
            source: "../images/scale_end.png"
            anchors{
                bottom: parent.bottom
                right: parent.right
            }
        }

        Label {
            id:scaleText
            color:"#004EAE"
            anchors.centerIn: parent
            text: "0 m"
        }

        Component.onCompleted: {
            map.computeScale()
        }



    }
    
    MapItemView{
        model: mav.uavModel
        delegate: MapQuickItem{
            coordinate:model.position
            sourceItem: Image {
                id: uavImage
                opacity: .75
                sourceSize.width:32
                sourceSize.height:32
                source: "../images/drone.png"
            }
            anchorPoint.x: uavImage.width/4
            anchorPoint.y: uavImage.height
        }
    }

    MapItemView{
        model:markerModel
        delegate:MapQuickItem{
            coordinate:model.position
            sourceItem: Image{
                id:waypointMarker
                opacity: .75
                sourceSize.width:80
                sourceSize.height:80
                source: "../images/marker.png"
            }
        anchorPoint.x: waypointMarker.width/2
        anchorPoint.y: waypointMarker.height/2
        }
    }

        MapItemView{
        model: boundingBoxModel
        delegate:MapQuickItem{
            coordinate:QtPositioning.coordinate(model.latitude, model.longitude)
            sourceItem: Image{
                id:waypointMarker
                opacity: .75
                sourceSize.width:80
                sourceSize.height:80
                source: "../images/marker-green.png"
            }
        anchorPoint.x: waypointMarker.width/2
        anchorPoint.y: waypointMarker.height/2
        }
    }

    // MapItemView{
    //     model:disksModel
    //     delegate:MapCircle{
    //         border.color: "red"
    //         border.width: 1
    //         center: QtPositioning.coordinate(model.latitude, model.longitude)
    //         radius: 53
    //     }
    // }

    
    // MapItemView{
    //     model:disksModel
    //     delegate:MapQuickItem{
    //          sourceItem: Image{
    //             id:waypointMarker
    //             opacity: .75
    //             sourceSize.width:80
    //             sourceSize.height:80
    //             source: "../images/marker.png"
    //         }
    //         coordinate: QtPositioning.coordinate(model.latitude, model.longitude)
    //         anchorPoint.x: waypointMarker.width/2
    //         anchorPoint.y: waypointMarker.height/2
    //     }
    // }

    MapItemView{
        id:transectItemView
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


    WaypointParamComponent{
        id:missionPopup

        onOkButtonClicked:{

            markerModel.append({"position":pos})
           // uavPath.addCoordinate(pos)
            console.log(pos.latitude + ", " + pos.longitude)

        }
    }


    DiskCoverageComponent{
        id:diskGenPopup

        onWaypointGenerated:{

            disksModel.clear()
            // create bounding box
            for (var i = 0; i < 4; i++)
            {
                boundingBoxModel.append(waypoints.get(i))
               
            }
            for (var i = 4; i < waypoints.count; i++)
            {
                disksModel.append(waypoints.get(i))

            }

            // clear waypoints
            waypoints.clear()     

        }
    }

    TransectComponent{
        id:transectPopup

        onWaypointGenerated:{  
            waypointModel.clear()               
            for(var i = 0; i < waypoints.count; i++){
                waypointModel.append(waypoints.get(i))
            }    
            waypoints.clear()           
            console.log(waypoints.count)
            console.log(waypointModel.count)
        }  
    }


    MouseArea{
        anchors.fill: parent
        focus: true
        hoverEnabled: true
        acceptedButtons: Qt.LeftButton

        onPressAndHold: {

            if(missionMode == 0)
            {
                waypoint = Qt.createQmlObject('import QtLocation 5.14;\MapCircle {radius: 5; color: "red"; opacity: 0.5; border.width: 0.5}', map)
                waypoint.center = map.toCoordinate(Qt.point(mouse.x, mouse.y))
                pos = map.toCoordinate(Qt.point(mouse.x,mouse.y));
                missionPopup.open()
               
            }

            if(missionMode == 1)
            {
                console.log("Swarm Mode")
                diskGenPopup.open()
            }

            else if(missionMode == 2)
            {
                console.log("Transect Mode")
                transectPopup.open()
            }

        }

    }

    Connections{
    target:appWindow
        function onClearMap() {

            waypointModel.clear()
            disksModel.clear()
            boundingBoxModel.clear()
            markerModel.clear()

            console.debug("Reset Map")
        }
    }
}


   

