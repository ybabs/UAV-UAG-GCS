import QtQuick 2.14
import QtQuick.Controls 2.14
import QtPositioning 5.14
import QtLocation 5.14
import QtGraphicalEffects 1.14
import QtQuick.Layouts 1.14
import QtQuick.Window 2.14

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

    plugin:Plugin{
        name:"esri"

        PluginParameter {
            name: "mapboxgl.mapping.items.insert_before"
            value: "aerialway"
        }

    }

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
        model: TSPModel
        delegate: MapPolyline{
            line.width: 3
            line.color: 'green'
            path: TSPModel.path

        }
    }

    // MapPolyline{
    //     id:pl
    //     line.width :20
    //     line.color: 'red'
    // }

    
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

    MapItemView{
        model:disksModel
        delegate:MapCircle{
            border.color: "red"
            border.width: 1
            center: QtPositioning.coordinate(model.latitude, model.longitude)
            radius: 53
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

    MouseArea{
        anchors.fill: parent
        focus: true
        hoverEnabled: true
        acceptedButtons: Qt.LeftButton

        onPressAndHold: {

            if(multiUavMode == false)
            {
                waypoint = Qt.createQmlObject('import QtLocation 5.14;\MapCircle {radius: 5; color: "red"; opacity: 0.5; border.width: 0.5}', map)
                waypoint.center = map.toCoordinate(Qt.point(mouse.x, mouse.y))
                pos = map.toCoordinate(Qt.point(mouse.x,mouse.y));
                missionPopup.open()
               
            }

            else
            {
                console.log("Swarm Mode")
                diskGenPopup.open()
            }

        }

    }

    //      function loadPath()
    //  {
    //         var lines = []
    //         for (var i = 0; i < TSPModel.geopath.size(); ++i)
    //         {
    //             lines[i] = TSPModel.geopath.coordinateAt(i)
    //         }
    //         return lines;
    // }

    //  Connections{
    //     target:TSPModel
    //     onGeopathChanged: {
    //         console.log("Path changed")
    //         pl.path = loadPath()
    //     }
    // }

    // Component.onCompleted: pl.path = loadPath()



}


   

