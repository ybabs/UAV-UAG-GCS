import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Controls.Material 2.15
import QtQuick.Controls.Styles 1.4


CheckBox{
    id:mavCheck

       property bool changeOnClick: true // or just emit clicked()

    MouseArea{
        anchors.fill: parent
        enabled: !mavCheck.changeOnClick
        onClicked: mavCheck.clicked();
    }
}