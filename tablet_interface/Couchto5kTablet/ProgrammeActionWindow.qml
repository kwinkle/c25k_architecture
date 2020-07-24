import QtQuick 2.4
import Ros 1.0

ProgrammeActionWindowForm {

    //TBC! note that programme actions which are styled differently to suggestion are duplicated (including their action id - which I think is ok?) as as refused action

    property double actionRemaining: 10000 //TO-DO: make this match csv/programme manager timing decisions for pre-loading programme actions
    property string previousState: "RESTART"
    property string currentState: "RESTART"

    property var suggestedAction
    property var programmeState
    property var stateDuration
    property var speed
    property var hr
    property var sessionDuration
    property var mood

    onSuggestedActionChanged: {
        console.log("programme action window suggested action triggered")
        if (suggestedAction.action === "RUN" || suggestedAction.action === "WALK") {
            currentState = suggestedAction.style
            programmeActionTimer.start()
        }

    }

    onCurrentStateChanged: {
        console.log("state changed")
        if (currentState === "RESTART" && previousState != "RESTART") {
            //turn off any prev. selected icons
            neutral.actionUnSelection.start()
            neutral.actionUnHighlight.start()
            neutral.timeBarAnimation.stop()
            neutral.timeBar.enabled = false
            neutral.timeBar.visible = false
            positive.actionUnSelection.start()
            positive.actionUnHighlight.start()
            positive.timeBarAnimation.stop()
            positive.timeBar.enabled = false
            positive.timeBar.visible = false
            challenging.actionUnSelection.start()
            challenging.actionUnHighlight.start()
            challenging.timeBarAnimation.stop()
            challenging.timeBar.enabled = false
            challenging.timeBar.visible = false
            sympathetic.actionUnSelection.start()
            sympathetic.actionUnHighlight.start()
            sympathetic.timeBarAnimation.stop()
            sympathetic.timeBar.enabled = false
            sympathetic.timeBar.visible = false
            previousState = "RESTART"
        }
        if (currentState === "NEUTRAL") {
            if (previousState === "RESTART") { //i.e. triggered by suggested action
                neutral.actionHighlight.start()
                neutral.timeBar.enabled = true
                neutral.timeBar.visible = true
                neutral.timeBarAnimation.start()
                previousState = "NEUTRAL"
            } else { //i.e. triggered by rejection of suggestion and choosing new style
                positive.actionUnHighlight.start()
                positive.timeBarAnimation.stop()
                positive.timeBar.enabled = false
                positive.timeBar.visible = false
                challenging.actionUnHighlight.start()
                challenging.timeBarAnimation.stop()
                challenging.timeBar.enabled = false
                challenging.timeBar.visible = false
                sympathetic.actionUnHighlight.start()
                sympathetic.timeBarAnimation.stop()
                sympathetic.timeBar.enabled = false
                sympathetic.timeBar.visible = false
            }
        } else if (currentState === "POSITIVE") {
            if (previousState === "RESTART") {
                positive.actionHighlight.start()
                positive.timeBar.enabled = true
                positive.timeBar.visible = true
                positive.timeBarAnimation.start()
                previousState = "POSITIVE"
            } else {
                neutral.actionUnHighlight.start()
                neutral.timeBarAnimation.stop()
                neutral.timeBar.enabled = false
                neutral.timeBar.visible = false
                challenging.actionUnHighlight.start()
                challenging.timeBarAnimation.stop()
                challenging.timeBar.enabled = false
                challenging.timeBar.visible = false
                sympathetic.actionUnHighlight.start()
                sympathetic.timeBarAnimation.stop()
                sympathetic.timeBar.enabled = false
                sympathetic.timeBar.visible = false
            }
        } else if (currentState === "CHALLENGING") {
            if (previousState === "RESTART") {
                challenging.actionHighlight.start()
                challenging.timeBar.enabled = true
                challenging.timeBar.visible = true
                challenging.timeBarAnimation.start()
                previousState = "CHALLENGING"
            } else {
                neutral.actionUnHighlight.start()
                neutral.timeBarAnimation.stop()
                neutral.timeBar.enabled = false
                neutral.timeBar.visible = false
                positive.actionUnHighlight.start()
                positive.timeBarAnimation.stop()
                positive.timeBar.enabled = false
                positive.timeBar.visible = false
                sympathetic.actionUnHighlight.start()
                sympathetic.timeBarAnimation.stop()
                sympathetic.timeBar.enabled = false
                sympathetic.timeBar.visible = false
            }
        } else if (currentState === "SYMPATHETIC") {
            if (previousState === "RESTART") {
                sympathetic.actionHighlight.start()
                sympathetic.timeBar.enabled = true
                sympathetic.timeBar.visible = true
                sympathetic.timeBarAnimation.start()
                previousState = "CHALLENGING"
            } else {
                neutral.actionUnHighlight.start()
                neutral.timeBarAnimation.stop()
                neutral.timeBar.enabled = false
                neutral.timeBar.visible = false
                positive.actionUnHighlight.start()
                positive.timeBarAnimation.stop()
                positive.timeBar.enabled = false
                positive.timeBar.visible = false
                challenging.actionUnHighlight.start()
                challenging.timeBarAnimation.stop()
                challenging.timeBar.enabled = false
                challenging.timeBar.visible = false
            }
        }

    }


    Timer {
        id: programmeActionTimer
        property string validation: "PASSIVE_ACCEPTED"
        interval: 1000; running: false; repeat: true
        onTriggered: {
            if (actionRemaining != 0) {
                actionRemaining = actionRemaining - 1000 //reduce time remaining on action
                programmeActionTime.text = (actionRemaining/1000)
            } else {
                //send validated, styled programme message
                console.log("publishing programme action");
                actionPublisher.action = suggestedAction.action;
                actionPublisher.style = currentState;
                actionPublisher.validation = validation;
                actionPublisher.publish();
                actionRemaining = 10000;
                validation = "PASSIVE_ACCEPTED";
                currentState = "RESTART";
                globalstates.state = "robotbehaviour";
                programmeActionTimer.stop();

            }
        }
    }

    Text {
        id: programmeActionTime
        visible: false
    }


    RosValidatedC25KPublisher{
        trigger_state_id: suggestedAction.trigger_state_id
        trigger_timestamp: suggestedAction.trigger_timestamp
        action_id: suggestedAction.action_id
        duration: suggestedAction.duration
        id: actionPublisher
        topic: "validated_actions"
    }

//    RosC25KActionSubscriber{
//        id:suggestedAction
//        topic: "unvalidated_actions"

    }

