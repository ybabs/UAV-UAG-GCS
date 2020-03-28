import QtQuick 2.14
import QtQuick.Window 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14
import QtLocation 5.14
import QtPositioning 5.14


Window{
    id:appWindow
    width:1280
    height:720
    visible:true

    property MapCircle point
    property MapCircle uav // make this a vector of points.
    property bool isPlay : true
    property string playColor: "green"
    property string pauseColor: "red"
    property string pauseText: "Pause"
    property string playText: "Play"

    Plugin {
        id:mapPlugin
        name: "osm"

    }

    MouseArea{
        anchors.fill:parent
        focus:true
        hoverEnabled:false
        acceptedButtons: Qt.LeftButton

        // onPressAndHold: {
                   

        // }
        onClicked:{

        }

    }

    Map{
        id: map
        anchors.fill:parent
        plugin: mapPlugin
        center: QtPositioning.coordinate(52.75591, -1.246310)
        zoomLevel:50

        MouseArea{
        anchors.fill:parent
        focus:true
        hoverEnabled:false
        acceptedButtons: Qt.LeftButton

         onPressAndHold: {
             point = Qt.createQmlObject('import QtLocation 5.9;\MapCircle {radius: 5; color: "red"; opacity: 0.5; border.width: 0.5}', map)
             point.center = map.toCoordinate(Qt.point(mouse.x, mouse.y))
             map.addMapItem(point)
             console.log(point.center.latitude)
             missionPopup.open()
            
                   

         }
        onClicked:{

        }

    }
       
    }

    StatusBar {
        id:statsRect
    }

    Waypointparam{
        id: missionPopup
    }

    Button{
        id:abortButton
        anchors {
            bottom: parent.bottom
            right: parent.right
            bottomMargin: 5
            rightMargin: 5
            
        }
        background: Rectangle{
             color: "red"
        }
        text: qsTr("Abort")
        opacity: 1

        onClicked:
        {
            planner.abortMission()
            console.log("abort clicked")
        }
    }

    Button{
        id:startButton
        anchors {
            bottom: parent.bottom
            right: abortButton.left
            bottomMargin: 5
            rightMargin: 5
            
        }
        background: Rectangle{
             color: "green"
        }
       
        text: qsTr("Start")
        opacity: 1

        onClicked:
        {
            planner.startMission()
            console.log("start clicked")
        }
    
    }

    Button{
        id:takeoffButton
        anchors{
            top: parent.top
            right: parent.right
            topMargin:5
            rightMargin:5
        }
        highlighted:true
        text: "Takeoff"
        
        onClicked: planner.takeoff()
    }

    Button{
        id:landButton
        anchors{
            top: takeoffButton.bottom
            right: parent.right
            topMargin:5
            rightMargin:5
        }
        text: "Land"
        highlighted:true

        onClicked: planner.land()
        
    }

    Button{
        id:rthButton
        anchors{
            top: landButton.bottom
            right: parent.right
            topMargin:5
            rightMargin:5
        }
        text: "RTH"
        highlighted:true
        onClicked: planner.goHome()
    }

    function setButtonColor()
    {
        var buttonState = playColor
        if(isPlay == true)
        {
            buttonState = playColor
        }

        else
        {
            buttonState = pauseColor
        }

        return buttonState
    }

    function setButtonText()
    {
        var buttonText = pauseText

        if(isPlay == true)
        {
            buttonText = playText
        }

        else
        {
            buttonText = pauseText
        }

        return buttonText

    }

    Button{
        id:pauseButton
       
        anchors{
            top:rthButton.bottom
            right:parent.right
            topMargin:5
            rightMargin:5
        }
        
  
        highlighted:true     

        text: setButtonText()
        
        onClicked:{
                 isPlay = !isPlay
                 planner.getPlayPause = isPlay
                
        }
        
    }
    
    Slider{
        id:speedSlider
        width:parent.width/6
        from:0
        to:15
        value:0
        stepSize:1
        enabled:true
        onValueChanged:
        {
            planner.getDroneSpeed = speedSlider.value
            speedText.text =  planner.getDroneSpeed
        }
    }

    Label {
        id: speedLabel
        text: qsTr("Speed: ") 
        horizontalAlignment: horizontalAlignment.CENTER
        //verticalAlignment: verticalAlignment.CENTER
        font.bold:false
        color:"steelblue"
        font.pixelSize: 20
        anchors{
            top:speedSlider.bottom
            left: parent.left
            topMargin:3
            leftMargin:8
        }
    }



    Text{
        id:speedText
        text: "0"
        color:"steelblue"
        font.bold:false
        font.pixelSize: 20
        anchors
        {
            top: speedSlider.bottom
            left: speedLabel.right
            topMargin:3
        }

    }

    Label{
        id:speedUinitLabel
        text:"m/s"
        color:"steelblue"
        font.bold:false
        font.pixelSize: 20
        anchors
        {
            top: speedSlider.bottom
            left: speedText.right
             topMargin:3
             leftMargin:1
        }

    }

    ColumnLayout {
        id:missionConfigLayout
        anchors
            {
                top: pauseButton.bottom
                right:parent.right
                topMargin: 10
                rightMargin:5

            }

        Label{
            id:missionEndLabel
            text: "Mission End "
            
        }

        RadioButton {
                id:noActionRadioButton
                checked: false
                 text: qsTr("Hover") 
                 onClicked:
                 {
                     planner.getHoverFlag = 1
                     planner.getRthFlag = 0
                     planner.getLandFlag = 0
                 }
            }
        RadioButton {
                id:rthRadioButton
                text: qsTr("RTH") 
                onClicked:
                 {
                     planner.getHoverFlag = 0
                     planner.getRthFlag = 1
                     planner.getLandFlag = 0
                 }
            }
        RadioButton {
                id: landRadioButton
                text: qsTr("Land") 
                onClicked:
                 {
                     planner.getHoverFlag = 0
                     planner.getRthFlag = 0
                     planner.getLandFlag = 1
                 }
            }
    }

}

//QML145