import QtQuick 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14


ToolBar{

    property bool busyIndicatorRunning: false
    property bool searchBarVisible: true
    property bool inPlay: true
    property bool swarmCheckState: false
    property string pauseText: "Pause"
    property string playText: "Play"

    signal startSearch(string searchCoord)
    signal searchTextChanged(string searchCoord)
    signal endSearchTextChanged(string searchCoord)
    signal waypointGenerated(ListModel waypoints)
    signal showMap()
    signal swarmModeChecked(bool value)

    ListModel{
        id: pointsList
    }


    onSearchBarVisibleChanged: {
        searchBar.opacity = searchBarVisible ? 1: 0
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

    function setButtonText()
    {
        var buttonText = pauseText
        if(inPlay === true)
        {
            buttonText = pauseText
        }

        else{
            buttonText = playText
        }

        return buttonText
    }

    RowLayout{
        id:searchBar
        width:parent.width
        height: parent.height
        Behavior on opacity {NumberAnimation{}}
        visible: opacity? true : false

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
            Layout.preferredWidth: parent.width * 0.08
            Layout.fillWidth: true
            Behavior on opacity { NumberAnimation{} }
            visible: opacity ? true : false
            placeholderText: "Speed"
            validator: IntValidator{bottom:1; top:15}
            onAccepted: {
                console.log(speedText.text + "m/s")
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

            }
        }

        Button{
            id:takeoffButton
            text:"Takeoff"
            onClicked: planner.takeoff()
        }

        Button{
            id:landButton
            text:"Land"
            onClicked: planner.land()
        }

        MissionConfigComponent{

        }

        MavComponent{

        }

        CheckBox{
            id:swarmCheckBox
            text: "Swarm Mode"
            onClicked:{
                if(checked)
                {
                    swarmCheckState = true
                }

                else
                {
                    swarmCheckState = false
                }

                swarmModeChecked(swarmCheckState)
            }
        }





    }

}
