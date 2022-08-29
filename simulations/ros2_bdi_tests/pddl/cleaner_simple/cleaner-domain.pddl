;; domain file: cleaner-domain.pddl

(define (domain cleaner-domain)

    (:requirements :strips :typing :fluents :durative-actions)

    ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:types
        waypoint
        robot
        void
    );; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

    (:predicates
        (in ?r - robot ?wp - waypoint)
        (workfree ?r - robot)
        (recharging_station ?wp - waypoint)
        (cleaned ?wp - waypoint)
        (full_recharged ?r - robot)
        (pred_a ?v - void)
        (pred_b ?v - void)
        (done ?v - void)
    )

    (:functions
        (battery_charge ?r - robot)
    )

    (:durative-action movetoward
        :parameters (?r - robot ?wp_from ?wp_to - waypoint)
        :duration (= ?duration 4)
        :condition (and
            (at start (in ?r ?wp_from))
            (at start (workfree ?r))
            (at start (> (battery_charge ?r) 15))
            (over all (> (battery_charge ?r) 10))
        )
        :effect (and
            (at start (not(workfree ?r)))
            (at start (not(in ?r ?wp_from)))
            (at end (in ?r ?wp_to))
            (at end (workfree ?r))
            (at end (decrease (battery_charge ?r) 10))
        )
    )

    (:durative-action doclean
        :parameters (?r - robot ?wp - waypoint)
        :duration (= ?duration 4)
        :condition (and
            (at start (workfree ?r))
            (at start (> (battery_charge ?r) 30))
            (over all (> (battery_charge ?r) 10))
            (over all (in ?r ?wp))
        )
        :effect (and
            (at start (not(workfree ?r)))
            (at end (workfree ?r))
            (at end (cleaned ?wp))
            (at end (decrease (battery_charge ?r) 20))
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
            (at end (assign (battery_charge ?r) 100))
            (at end (full_recharged ?r))
        )
    )

     (:durative-action do_nothing
        :parameters (?r - robot ?v - void)
        :duration (= ?duration 4)
        :condition (and
            (at start (workfree ?r))
        )
        :effect (and
            (at start (not(workfree ?r)))
            (at end (workfree ?r))
            (at end (done ?v))
        )
    )

)
