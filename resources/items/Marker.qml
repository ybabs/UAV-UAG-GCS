import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtLocation 5.15
import QtPositioning 5.15
import QtGraphicalEffects 1.15



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