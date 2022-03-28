;; domain file: blocks-domain.pddl

(define (domain blocks-domain)

    (:requirements :strips :typing :fluents :durative-actions)

    ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:types
        box stackbase - box_or_stackbase
        gripper
    );; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

    (:predicates
        (upon ?g - gripper ?s - box_or_stackbase)
        (on ?b1 - box ?b2 - box_or_stackbase)
        (clear ?b - box_or_stackbase)
        (holding ?g - gripper ?b - box)
        (free ?g - gripper)
    )

    (:functions
        
    )

    (:durative-action pickup
        :parameters (?g - gripper ?b1 - box ?bs2 - box_or_stackbase)
        :duration (= ?duration 2)
        :condition (and
            (at start (on ?b1 ?bs2))
            (at start (clear ?b1))
            (at start (upon ?g ?b1))
            (at start (free ?g))
        )
        :effect (and
            (at end (not(free ?g)))
            (at end (holding ?g ?b1))
            (at end (not(clear ?b1)))
            (at end (not(on ?b1 ?bs2)))
            (at end (clear ?bs2))
        )
    )

    (:durative-action putdown
        :parameters (?g - gripper ?b1 - box ?bs2 - box_or_stackbase)
        :duration (= ?duration 2)
        :condition (and
            (at start (holding ?g ?b1))
            (at start (clear ?bs2))
            (at start (upon ?g ?bs2))
        )
        :effect (and
            (at end (free ?g))
            (at end (not (holding ?g ?b1)))
            (at end (not(clear ?bs2)))
            (at end (clear ?b1))
            (at end (on ?b1 ?bs2))
        )
    )

    (:durative-action move
        :parameters (?g - gripper ?bs1 ?bs2 - box_or_stackbase)
        :duration (= ?duration 4)
        :condition (and
            (at start (upon ?g ?bs1))
            (over all (clear ?bs1))
            (over all (clear ?bs2))
        )
        :effect (and
            (at start (not(upon ?g ?bs1)))
            (at end (upon ?g ?bs2))
        )
    )
)
