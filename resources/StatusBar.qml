import QtQuick 2.14
import QtQuick.Window 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14

Rectangle {
        width: 500
        height:50
        color:"#7C807D"
        radius: 10
        opacity: 0.2

        anchors{
            bottomMargin:5
            bottom: parent.bottom
            left: parent.left
            leftMargin:5
        }


        Text{
            id:clientText
        anchors{
            left:parent.left
            verticalCenter: parent.verticalCenter
            leftMargin: 5
            //top:parent.center
            //topMargin:20

            
        }
        text:qsTr("Connected UAVs:")
        color: "black"
        font.pixelSize:16
        }

        Text{
            id:numDroneText
        anchors{
        
            left:clientText.right
            verticalCenter: parent.verticalCenter
            leftMargin: 2
            
        }
        text:qsTr("N/A")
        color: "black"
        font.pixelSize:16
        }

        Text{
            id:m100BatText
        anchors{
        
            left:numDroneText.right
            verticalCenter: parent.verticalCenter
            leftMargin: 5
            
        }
        text:qsTr("Bat 1:")
        color: "black"
        font.pixelSize:16
        }

        Text{
            id:m100BatValue
        anchors{
        
            left:m100BatText.right
            verticalCenter: parent.verticalCenter
            leftMargin: 2
            
        }
        text:qsTr("N/A")
        color: "black"
        font.pixelSize:16
        }

        Text{
            id:n3BatText
        anchors{
        
            left:m100BatValue.right
            verticalCenter: parent.verticalCenter
            leftMargin: 5
            
        }
        text:qsTr("Bat 2:")
        color: "black"
        font.pixelSize:16
        }

        Text{
            id:n3BatValue
        anchors{
        
            left:n3BatText.right
            verticalCenter: parent.verticalCenter
            leftMargin: 2
            
        }
        text:qsTr("N/A")
        color: "black"
        font.pixelSize:16
        }

        Text{
            id:homeDistText
            anchors{
                left:n3BatValue.right
                verticalCenter: parent.verticalCenter
                leftMargin: 5

            }

            text:qsTr("D1:")
            color: "black"
            font.pixelSize:16
        }

        Text{
            id:homeDistVal
            anchors{
                left:homeDistText.right
                verticalCenter: parent.verticalCenter
                leftMargin: 5

            }

            text:qsTr("N/A:")
            color: "black"
            font.pixelSize:16
        }

        Text{
            id:n3HomeDistText
            anchors{
                left:homeDistVal.right
                verticalCenter: parent.verticalCenter
                leftMargin: 5

            }

            text:qsTr("D2:")
            color: "black"
            font.pixelSize:16
        }

        Text{
            id:n3homeDistVal
            anchors{
                left:n3HomeDistText.right
                verticalCenter: parent.verticalCenter
                leftMargin: 5

            }

            text:qsTr("N/A:")
            color: "black"
            font.pixelSize:16
        }
        

    }

    
