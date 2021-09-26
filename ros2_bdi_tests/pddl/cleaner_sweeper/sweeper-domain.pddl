;; domain file: cleaner-domain.pddl

(define (domain cleaner-domain)

    (:requirements :strips :typing :fluents :durative-actions)

    ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:types
        waypoint
        robot
    );; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

    (:predicates
        (in ?r - robot ?wp - waypoint)
        (workfree ?r - robot)
        (recharging_station ?wp - waypoint)
        (swept ?wp - waypoint)
    )

    (:functions
        (battery_charge)
    )

    (:durative-action movetoward
        :parameters (?r - robot ?wp_from ?wp_to - waypoint)
        :duration (= ?duration 4)
        :condition (and
            (at start (in ?r ?wp_from))
            (at start (workfree ?r))
            (at start (> (battery_charge) 20))
            (over all (> (battery_charge) 10))
        )
        :effect (and
            (at start (not(workfree ?r)))
            (at end (workfree ?r))
            (at start (not(in ?r ?wp_from)))
            (at end (in ?r ?wp_to))
            (at end (decrease (battery_charge) 10))
        )
    )

    (:durative-action dosweep
        :parameters (?r - robot ?wp - waypoint)
        :duration (= ?duration 4)
        :condition (and
            (at start (workfree ?r))
            (at start (> (battery_charge) 30))
            (over all (> (battery_charge) 10))
            (over all (in ?r ?wp))
        )
        :effect (and
            (at start (not(workfree ?r)))
            (at end (workfree ?r))
            (at end (swept ?wp))
            (at end (decrease (battery_charge) 20))
        )
    )

    (:durative-action recharge
        :parameters (?r - robot ?wp - waypoint)
        :duration (= ?duration 4)
        :condition (and
            (at start (workfree ?r))
            (over all (in ?r ?wp))
            (over all (recharging_station ?wp))
        )
        :effect (and
            (at start (not(workfree ?r)))
            (at end (workfree ?r))
            (at end (assign (battery_charge) 100))
        )
    )

)
