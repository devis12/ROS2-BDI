;; domain file: carrier-domain.pddl

(define (domain carrier-domain)

    (:requirements :strips :typing :fluents :durative-actions :negative-preconditions)

    ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:types
        base_wp deposit_wp - waypoint
        carrier
        box
    );; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

    (:predicates
        ( in ?c - carrier ?wp - waypoint)
        ; ( carrying ?c - carrier ?b - box )
        ( fully_loaded ?c - carrier)
        ( fully_unloaded ?c - carrier)
    )

    (:functions
        (moving_boxes ?c - carrier)
    )

    (:durative-action carrier_move
        :parameters (?c - carrier ?wp1 ?wp2 - waypoint)
        :duration (= ?duration 6)
        :condition (and
            (at start (in ?c ?wp1))
        )
        :effect (and
            (at start (not(in ?c ?wp1)))
            (at end (in ?c ?wp2))
        )
    )

    (:durative-action carrier_unload
        :parameters (?c - carrier ?dep_wp - deposit_wp)
        :duration (= ?duration 2)
        :condition (and
            (at start (> (moving_boxes ?c) 0))
            (at start (fully_loaded ?c))
            (at start (in ?c ?dep_wp))
            (over all (in ?c ?dep_wp))
        )
        :effect (and
            (at start (not(fully_loaded ?c)))
            (at end (fully_unloaded ?c))
            ; (at start 
            ;     ; (forall 
            ;     ;     (?b - box)
            ;     ;         (imply (carrying ?c ?b) (and(not(carrying ?c ?b))))  
            ;     ; )
            ; )
            (at end (assign (moving_boxes ?c) 0))
        )
    )

)
