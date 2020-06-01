import QtQuick 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14
import QtQuick.Controls.Material 2.14

    RowLayout{

        CheckBox{
            id: mav1checkBox
            text: "MAV 1"
            font.bold:false
            onClicked: {
                if(checked)
                {
                    // TODO Implement logic here
                }
            }

        }

        CheckBox{
            id: mav2checkBox
            text: "MAV 2"
            onClicked: {
                if(checked)
                {
                    // TODO Implement logic here
                }
            }

        }

        CheckBox{
            id: mav3checkBox
            text: "MAV 3"
            onClicked: {
                if(checked)
                {
                    // TODO Implement logic here
                }
            }

        }

        CheckBox{
             id: mav4checkBox
             text: "MAV 4"
             onClicked: {
                 if(checked)
                 {
                     // TODO Implement logic here
                 }
             }

        }

        Button{
            id: setMavControlButton
            text: "Set Control"
            onClicked: {

                // Upload data here

            }

        }

        Button{
            id:setAllControlButton
            text:"Control All"
            onClicked: {

                mav1checkBox.checked = true;
                mav2checkBox.checked = true;
                mav3checkBox.checked = true;
                mav4checkBox.checked = true;

                // set Logic here



            }

        }

    }

