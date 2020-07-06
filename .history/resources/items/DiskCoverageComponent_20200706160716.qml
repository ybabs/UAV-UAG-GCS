import QtQuick 2.14
import QtQuick.Window 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14

Popup
{
    x: (parent.width - width) / 2
    y: (parent.height - height) / 2
    height: 230
    visible: false
    focus:true

    TextField{
        id: centerSearchText
        Behavior on opacity {NumberAnimation{}}
        property bool ignoreTextChange: false
        placeholderText: qsTr("Enter center coordinate")
        Layout.fillWidth:true
        onTextChanged:{
            if(!ignoreTextChange){
                searchTextChanged(text)
            }
        }
        onAccepted: startSearch(text) 
    }





     TextField{
            id:searchText
            Behavior on opacity {NumberAnimation{}}
            property bool ignoreTextChange : false
            placeholderText: qsTr("Enter Start Coordinate")
            Layout.fillWidth: true
            onTextChanged: {
                if(!ignoreTextChange){
                    searchTextChanged(text)
                }
            }
            onAccepted: startSearch(text)
        }

}