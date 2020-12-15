import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Popup
{
    x: (parent.width - width) / 2
    y: (parent.height - height) / 2
    height: 250
    width: 250
    visible: false
    focus:true

    signal startSearch(string searchCoord)
    signal searchTextChanged(string searchCoord)
    signal endSearchTextChanged(string searchCoord)
    signal waypointGenerated(ListModel waypoints)

    ListModel{
        id: pointsList
    }

    function showSearch(text) {
        if(text !== null){
            searchText.ignoreTextChange = true
            searchText.text = text
            searchText.ignoreTextChange = false

        }
    }

    function showEndSearch(text){
        if(text !== null){
            endSearchText.ignoreTextChange = true
            endSearchText.text = text
            endSearchText.ignoreTextChange = false

        }

    }

    ColumnLayout
    {

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

        TextField{
            id:endSearchText
            Behavior on opacity {NumberAnimation{}}
            property bool ignoreTextChange : false
            placeholderText: qsTr("Enter End Coordinate")
            Layout.fillWidth: true
            onTextChanged: {
                if(!ignoreTextChange){
                    endSearchTextChanged(text)
                }
            }
            onAccepted: startSearch(text)
        }

        TextField{
            Layout.fillWidth: true
            id:waypointText
            Behavior on opacity {NumberAnimation{}}
            property bool ignoreTextChange : false
            placeholderText: qsTr("Number of Coordinates")
            validator: IntValidator {bottom:0; top:100}
            onTextChanged: {
            }
            onAccepted: {
            }
        }

        TextField{
            id:speedText
            Layout.fillWidth: true
            Behavior on opacity { NumberAnimation{} }
            visible: opacity ? true : false
            placeholderText: "Speed"
            validator: IntValidator{bottom:1; top:15}
            onAccepted: {
                planner.droneSpeed = speedText.text
                console.log(planner.droneSpeed + "m/s")
            }
        }


        Button{
            id: generateWpButton
            text:"Generate"
            onClicked:{
                planner.addGeneratedWaypoints(searchText.text, endSearchText.text, waypointText.text)

                for(var i in planner.trackpoints){
                    var p = planner.trackpoints[i];
                   pointsList.append(p)
                }

                waypointGenerated(pointsList) 
                transectPopup.close()              

            }
        }
        
    }

}