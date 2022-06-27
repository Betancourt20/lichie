

skillset manipulator{
    resource {
        arm_status {
            initial TRANSPORTATION
            TRANSPORTATION -> READY_TO_DEPLOY
            READY_TO_DEPLOY -> NEXT_TO_TARGET
            NEXT_TO_TARGET -> HITTING_TARGET
            HITTING_TARGET -> NEXT_TO_TARGET
            DEPLOYMENT -> READY_TO_DEPLOY
            READY_TO_DEPLOY -> TRANSPORTATION
        }

        target_status{
            initial NO_TARGET
            NO_TARGET -> TARGET_SEEN
            TARGET_SEEN -> TARGET_DOWN
        }

        control_mode{
        initial MANUAL_MODE
        MANUAL_MODE -> AUTO_MODE
        AUTO_MODE -> MANUAL_MODE
        }

    }

    event {
        to_auto_from_manual {
            guard control_mode == MANUAL_MODE
            control_mode -> AUTO_MODE
        }

        to_manual_from_auto {
            guard control_mode == AUTO_MODE
            control_mode -> MANUAL_MODE
        }

        target_spotted {
            guard target_status == NO_TARGET
            target_status -> TARGET_SEEN
        }

        target_is_down {
            guard target_status == TARGET_SEEN
            target_status -> TARGET_DOWN
        }

        arm_next_to_target {
            guard arm_status == HITTING_TARGET
            arm_status -> NEXT_TO_TARGET
        }

    }

    skill ready_arm {

        precondition {
             is_not_ready {
                guard arm_status == TRANSPORTATION
            }

             is_auto {
                control_mode == AUTO_MODE
            }

            is_seen {
                target_status == TARGET_SEEN
            }
        }

        invariant {
            is_auto {
                guard control_mode == AUTO_MODE
            }
        }

        success is_ready {
            arm_status -> READY_TO_DEPLOY
        }

        failure cannot_deploy
    }

    skill move_arm {

        precondition {
             is_not_ready {
                guard arm_status == READY_TO_DEPLOY
            }

             is_auto {
                control_mode == AUTO_MODE
            }

            is_seen {
                target_status == TARGET_SEEN
            }
        }

        invariant {
            is_auto {
                guard control == AUTO_MODE
            }
        }

        success is_ready {
            arm_status -> NEXT_TO_TARGET
        }

        failure cannot_deploy
    }

    skill hit_target_arm {

        precondition {
             is_not_ready {
                guard arm_status == NEXT_TO_TARGET
            }

             is_auto {
                control_mode == AUTO_MODE
            }

            is_seen {
                target_status == TARGET_SEEN
            }
        }

        start {
            arm_status -> HITTING_TARGET
        }

        invariant {
            is_auto {
                guard control == AUTO_MODE
                failure {
                arm_status -> NEXT_TO_TARGET
                }
            }
        }

        success is_ready {
            arm_status -> NEXT_TO_TARGET
            target_status -> TARGET_DOWN
        }

        failure cannot_deploy {
            arm_status -> NEXT_TO_TARGET
        }

    }

    skill return_to_ready_to_transportation {

        precondition {
             is_not_ready {
                guard arm_status == NEXT_TO_TARGET
            }

             is_auto {
                control_mode == AUTO_MODE
            }
        }

        start {
            arm_status -> READY_TO_DEPLOY
        }

        invariant {
            is_auto {
                guard control == AUTO_MODE
                failure {
                arm_status -> TRANSPORTATION
                }
            }
        }

        success is_ready {
            arm_status -> TRANSPORTATION
        }

    }


}
