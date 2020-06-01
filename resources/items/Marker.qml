import QtQuick 2.14
import QtQuick.Window 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14
import QtLocation 5.14
import QtPositioning 5.14
import QtGraphicalEffects 1.14



MapItemView{
    id:interpolatedMarkersView

    delegate:MapQuickItem{
        coordinate: QtPositioning.coordinate(lat, lon)
        sourceItem: Image{
            id:wpMarker
            opacity: .75
            sourceSize.width:80
            sourceSize.height:80
            source: "qrc:///marker-red.png"
        }
    anchorPoint.x: wpMarker.width/2
    anchorPoint.y: wpMarker.height/2
    }
}