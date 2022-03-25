;; domain file: blocks-domain.pddl

(define (domain blocks-domain)

    (:requirements :strips :typing :fluents :durative-actions)

    ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:types
        stack
        box floor - box_or_floor
        gripper
    );; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

    (:predicates
        (upon ?g - gripper ?s - stack)
        (on ?b1 - box ?b2 - box_or_floor)
        (clear ?b - box_or_floor)
        (in ?b - box_or_floor ?s - stack)
        (holding ?g - gripper ?b - box)
        (free ?g - gripper)
    )

    (:functions
        
    )

    (:durative-action pickup
        :parameters (?g - gripper ?b1 - box ?bf2 - box_or_floor ?s - stack)
        :duration (= ?duration 2)
        :condition (and
            (at start (in ?b1 ?s))
            (at start (on ?b1 ?bf2))
            (at start (clear ?b1))
            (at start (upon ?g ?s))
            (at start (free ?g))
        )
        :effect (and
            (at end (not(free ?g)))
            (at end (holding ?g ?b1))
            (at end (not(clear ?b1)))
            (at end (not(on ?b1 ?bf2)))
            (at end (not(in ?b1 ?s)))
            (at end (clear ?bf2))
        )
    )

    (:durative-action putdown
        :parameters (?g - gripper ?b1 - box ?bf2 - box_or_floor ?s - stack)
        :duration (= ?duration 2)
        :condition (and
            (at start (holding ?g ?b1))
            (at start (in ?bf2 ?s))
            (at start (clear ?bf2))
            (at start (upon ?g ?s))
        )
        :effect (and
            (at end (free ?g))
            (at end (not (holding ?g ?b1)))
            (at end (not(clear ?bf2)))
            (at end (clear ?b1))
            (at end (on ?b1 ?bf2))
            (at end (in ?b1 ?s))
        )
    )

    (:durative-action move
        :parameters (?g - gripper ?s1 ?s2 - stack)
        :duration (= ?duration 4)
        :condition (and
            (at start (upon ?g ?s1))
        )
        :effect (and
            (at start (not(upon ?g ?s1)))
            (at end (upon ?g ?s2))
        )
    )
)
