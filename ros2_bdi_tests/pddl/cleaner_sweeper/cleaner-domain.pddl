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
        (cleaned ?wp - waypoint)
        (swept ?wp - waypoint)
        (waitforsweep ?wp - waypoint)
        (issweeper ?r_sweeper - robot)
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
            (at start (> (battery_charge) 40))
            (over all (> (battery_charge) 20))
        )
        :effect (and
            (at start (not(workfree ?r)))
            (at start (not(in ?r ?wp_from)))
            (at end (in ?r ?wp_to))
            (at end (workfree ?r))
            (at end (decrease (battery_charge) 20))
        )
    )

    (:durative-action asksweeping
        :parameters (?r_me ?r_sweeper - robot ?wp - waypoint)
        :duration (= ?duration 16)
        :condition (and
            (over all (issweeper ?r_sweeper))
        )
        :effect (and
            (at start (waitforsweep ?wp))
            (at end (not(waitforsweep ?wp)))
            (at end (swept ?wp))
        )
    )

    (:durative-action doclean
        :parameters (?r - robot ?wp - waypoint)
        :duration (= ?duration 4)
        :condition (and
            (at start (workfree ?r))
            (at start (swept ?wp))
            (at start (> (battery_charge) 50))
            (over all (> (battery_charge) 25))
            (over all (in ?r ?wp))
            (over all (swept ?wp))
        )
        :effect (and
            (at start (not(workfree ?r)))
            (at end (workfree ?r))
            (at end (cleaned ?wp))
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
