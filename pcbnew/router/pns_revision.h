/*
* REVISION - helper class for the KiRouter
*
* Copyright (C) 2016 KiCad Developers, see AUTHORS.txt for contributors.
* Author: Michael Steinberg <michsteinb@gmail.com>
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the
* Free Software Foundation, either version 3 of the License, or (at your
* option) any later version.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef PNS_DIFF_STATE_H__
#define PNS_DIFF_STATE_H__

#include <memory>
#include <vector>

namespace PNS {

    /// REVISION Overview
    /// REVISIONs track the add-/remove-ITEM actions in form of a revision-tree and
    /// manage the lifetime of the added ITEMs.
    /// One can branch every revision, drop branches, squash changes into parent revisions etc.
    /// The class helps implementing revisions of states with a dynamic set of immutable items,
    /// as is the case with the spatial state of PNS::NODE, that is used throughout to implement
    /// the push&shove routing.
    /// -> Could possibly be made a template.

    class ITEM;
    class REVISION;

    ///
    /// Class SEQUENCE
    /// Wraps up a [start, end) iterator sequence.
    ///

    template< typename ITERATOR >
    class SEQUENCE
    {
    public:
        SEQUENCE( ITERATOR aBegin, ITERATOR aEnd )
            : m_begin( std::move( aBegin ) ), m_end( std::move( aEnd ) )
        {}

        SEQUENCE( SEQUENCE&& ) = default;
        SEQUENCE& operator=( SEQUENCE&& ) = default;

        size_t size() const {
            return std::distance( m_begin, m_end );
        }

        const ITERATOR& begin() {
            return m_begin;
        }

        const ITERATOR& end() {
            return m_end;
        }

    private:
        ITERATOR m_begin, m_end;
    };

    ///
    /// Class REVISION_PATH
    ///

    class REVISION_PATH {
    public:
        using REVERT_ITERATOR = std::vector< const REVISION* >::const_iterator;
        using APPLY_ITERATOR  = std::vector< const REVISION* >::const_reverse_iterator;

        REVISION_PATH( std::vector< const REVISION*> aRevertList,
                       std::vector< const REVISION*> aApplyList );
        REVISION_PATH( REVISION_PATH&& ) = default;
        REVISION_PATH( const REVISION_PATH& ) = default;
        REVISION_PATH& operator=( REVISION_PATH&& ) = default;
        REVISION_PATH& operator=( const REVISION_PATH& ) = default;

        void Invert();
        size_t Size() const;

        // these need to be reverted in order
        SEQUENCE<REVERT_ITERATOR> RevertSequence() const;

        // these need to be applied in order
        SEQUENCE<APPLY_ITERATOR> ApplySequence() const;

    private:
        std::vector< const REVISION* > m_revert;  // upwards
        std::vector< const REVISION* > m_apply;   // upwards
    };

    ///
    /// Class CHANGE_SET
    /// Holds aggregated changes over multiple revisions, non-owned items
    /// Can be generated out of a revision-path.
    ///

    class CHANGE_SET {
    private:
        using ITEMS_CONTAINER   = std::vector< ITEM* >;

    public:
        using ITEM_ITERATOR       = ITEMS_CONTAINER::iterator;
        using CONST_ITEM_ITERATOR = ITEMS_CONTAINER::const_iterator;

    public:
        void Clear();
        void Apply ( const REVISION* aState );
        void Revert( const REVISION* aState );

        void Add   ( ITEM* aItem );
        void Remove( ITEM* aItem );

        static CHANGE_SET FromPath( const REVISION_PATH& aPath );

        SEQUENCE< ITEM_ITERATOR >       AddedItems();
        SEQUENCE< CONST_ITEM_ITERATOR > AddedItems() const;

        SEQUENCE< ITEM_ITERATOR >       RemovedItems();
        SEQUENCE< CONST_ITEM_ITERATOR > RemovedItems() const;

    private:
        ITEMS_CONTAINER m_added_items;
        ITEMS_CONTAINER m_removed_items;
    };

    ///
    /// Class REVISION
    /// Tracks differences of world revisions and manages all ITEM's lifetimes
    ///

    class REVISION {
    public:
        REVISION();
        ~REVISION();

    private:
        using ADDED_ITEMS_CONTAINER   = std::vector< std::unique_ptr< ITEM > >;
        using REMOVED_ITEMS_CONTAINER = std::vector< ITEM* >;
        using BRANCHES_CONTAINER      = std::vector< std::unique_ptr< REVISION > >;

    public:
        using ADDED_ITEM_ITERATOR         = ADDED_ITEMS_CONTAINER::iterator;
        using CONST_ADDED_ITEM_ITERATOR   = ADDED_ITEMS_CONTAINER::const_iterator;

        using REMOVED_ITEM_ITERATOR       = REMOVED_ITEMS_CONTAINER::iterator;
        using CONST_REMOVED_ITEM_ITERATOR = REMOVED_ITEMS_CONTAINER::const_iterator;

        using BRANCH_ITERATOR             = BRANCHES_CONTAINER::iterator;
        using CONST_BRANCH_ITERATOR       = BRANCHES_CONTAINER::const_iterator;

    public:
        /**
         * Function AddItem
         * Add an item to this revision. The revision tree takes ownership.
         * @note Undefined behaviour if this revision is not a leaf.
         * @param aItem to be added to the revision tree
         */
        void AddItem( std::unique_ptr< ITEM > aItem );

        /**
         * Function RemoveItem
         * Remove aItem if it was added in this revision or shadow it otherwise
         * @note Undefined behaviour if this revision is not a leaf.
         * @param aItem to be be removed or shadowed in this revision's sub-tree
         */
        void RemoveItem( ITEM* aItem );

        /**
         * Function IsShadowed
         * Checks if aItem is alive but shadowed (removed in a revision including this one)
         * @return true iff the item is shadowed
         */
        bool IsShadowed( const ITEM* aItem );

        bool Owns( const ITEM* aItem ) const;

        size_t Depth() const;

        CHANGE_SET GetRevisionChanges() const;
        REVISION* Revert();
        void Clear();

        /**
        * Function Squash
        * Squash this revision into parent.
        * @note Undefined behaviour if this revision is a root revision
        * @return REVISION* - parent of this revision.
        * @pre  Parent() != nullptr
        * @post this revision is deleted
        */
        REVISION* Squash();

        /**
        * Function ClearBranches
        * Remove all branches of this revision, thereby deleting all items introduced
        * below this revision.
        * @post Branches().size() == 0
        */
        void ClearBranches();

        /**
         * Function Branch
         * Create a new branch of this revision and return a pointer to it.
         * @return REVISION* pointer to the created branch
         * @post retval->Parent() == this
         */
        REVISION* Branch();

        /**
         * Function RemoveBranch
         * Remove a branch from this revision.
         * @note Undefined behaviour if aBranch is not a branch of this revision.
         * @param aBranch Pointer to a branch
         */
        void RemoveBranch( REVISION* aBranch );

        /**
         * Function ReleaseBranch
         * Release a branch from this revision and return an owning pointer to it.
         * @note Undefined behaviour if aBranch is not a branch of this revision.
         * @param aBranch Pointer to a branch
         * @return Owning pointer to the former branch.
         */
        std::unique_ptr< REVISION > ReleaseBranch( const REVISION* aBranch );

        /**
         * Function Parent
         * Returns the parent of this branch or nullptr if this revision is a root
         * @return Const pointer to parent revision or nullptr if root
         */
        REVISION* Parent() const;

        /**
         * Function NumChanges
         * Return the aggregate number of individual non-cancelling changes in this revision
         * @return Aggregate number of individual non-cancelling changes in this revision
         */
        size_t NumChanges() const;

        /**
         * Function Path
         * Returns a path of DIFF_STATEs from this revision to an ancestor revion. Paths are
         * always directed towards the root of the revision tree.
         * @note Undefined behaviour if aAncestor is not an ancestor of this revision
         * @pre Parent()[->Parent()*] == aAncestor
         */
        REVISION_PATH Path( const REVISION* aAncestor ) const;


        //
        // Sequence accessors for { added items; removed items; branches }
        //
        SEQUENCE< ADDED_ITEM_ITERATOR >       AddedItems();
        SEQUENCE< CONST_ADDED_ITEM_ITERATOR > AddedItems() const;

        SEQUENCE< REMOVED_ITEM_ITERATOR >       RemovedItems();
        SEQUENCE< CONST_REMOVED_ITEM_ITERATOR > RemovedItems() const;

        SEQUENCE< BRANCH_ITERATOR >       Branches();
        SEQUENCE< CONST_BRANCH_ITERATOR > Branches() const;
    private:
        void Absorb( REVISION* aDiff );

        REVISION* m_parent;

        std::vector< std::unique_ptr< REVISION > > m_branches;

        std::vector< std::unique_ptr< ITEM > > m_added_items;
        std::vector< ITEM* >                   m_removed_items;
    };

    REVISION_PATH Path( const REVISION* aFrom, const REVISION* aTo );



}

#endif
