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

#include <cassert>
#include <algorithm>
#include "pns_item.h"
#include "pns_revision.h"

namespace PNS {

    //
    // CHANGE_SET
    //
    void CHANGE_SET::Clear()
    {
        m_added_items.clear();
        m_removed_items.clear();
    }

    void CHANGE_SET::Apply( const REVISION* aState )
    {
        for( auto& item : aState->AddedItems() )
        {
            Add( item.get() );
        }

        for( auto item : aState->RemovedItems() )
        {
            Remove( item );
        }
    }

    void CHANGE_SET::Revert( const REVISION* aState )
    {
        for( auto& item : aState->AddedItems() )
        {
            Remove( item.get() );
        }

        for( auto item : aState->RemovedItems() )
        {
            Add( item );
        }
    }

    void CHANGE_SET::Add( ITEM* aItem )
    {
        auto it = std::find( m_removed_items.begin(), m_removed_items.end(), aItem );
        if( it != m_removed_items.end() )
        {
            m_removed_items.erase( it );
        }
        else
        {
            m_added_items.push_back( aItem );
        }
    }

    void CHANGE_SET::Remove( ITEM* aItem )
    {
        auto it = std::find( m_added_items.begin(), m_added_items.end(), aItem );

        if( it != m_added_items.end() )
        {
            m_added_items.erase( it );
        }
        else
        {
            m_removed_items.push_back( aItem );
        }
    }

    CHANGE_SET CHANGE_SET::FromPath( const REVISION_PATH& aPath )
    {
        CHANGE_SET result;

        for( auto& revert : aPath.RevertSequence() )
        {
            result.Revert( revert );
        }

        for( auto& apply : aPath.ApplySequence() )
        {
            result.Apply( apply );
        }

        return result;
    }

    SEQUENCE< CHANGE_SET::ITEM_ITERATOR > CHANGE_SET::AddedItems()
    {
        return{ m_added_items.begin(), m_added_items.end() };
    }

    SEQUENCE< CHANGE_SET::CONST_ITEM_ITERATOR > CHANGE_SET::AddedItems() const
    {
        return{ m_added_items.begin(), m_added_items.end() };
    }

    SEQUENCE< CHANGE_SET::ITEM_ITERATOR > CHANGE_SET::RemovedItems()
    {
        return{ m_removed_items.begin(), m_removed_items.end() };
    }

    SEQUENCE< CHANGE_SET::CONST_ITEM_ITERATOR > CHANGE_SET::RemovedItems() const
    {
        return{ m_removed_items.begin(), m_removed_items.end() };
    }

    //
    // REVISION_PATH
    //

    REVISION_PATH::REVISION_PATH( std::vector<const REVISION*> aRevertList,
                                  std::vector<const REVISION*> aApplyList ) :
        m_revert( std::move( aRevertList ) ), m_apply( std::move( aApplyList ) )
    {
    }

    void REVISION_PATH::Invert()
    {
        std::swap( m_revert, m_apply );
    }

    size_t REVISION_PATH::Size() const
    {
        return m_revert.size() + m_apply.size();
    }

    SEQUENCE<REVISION_PATH::REVERT_ITERATOR> REVISION_PATH::RevertSequence() const
    {
        return{ m_revert.begin(), m_revert.end() };
    }

    SEQUENCE<REVISION_PATH::APPLY_ITERATOR> REVISION_PATH::ApplySequence() const
    {
        return{ m_apply.rbegin(), m_apply.rend() };
    }

    //
    // DiffState
    //

    REVISION::REVISION()
        : m_parent( nullptr )
    {
    }

    REVISION::~REVISION()
    {
    }

    void REVISION::Clear()
    {
        m_added_items.clear();
        m_removed_items.clear();
        m_branches.clear();
    }

    CHANGE_SET REVISION::GetRevisionChanges() const
    {
        CHANGE_SET changes;
        changes.Apply( this );
        return changes;
    }

    void REVISION::AddItem( std::unique_ptr< ITEM > aItem )
    {
        aItem->SetOwner( this );
        m_added_items.push_back( std::move( aItem ) );
    }

    void REVISION::RemoveItem( ITEM* aItem )
    {
        auto it = std::find_if(
            m_added_items.begin(),
            m_added_items.end(),
            [aItem]( const std::unique_ptr< ITEM >& ptr ) { return ptr.get() == aItem; }
        );

        if( it != m_added_items.end() )
        {
            m_added_items.erase( it );
        }
        else
        {
            // maybe check if the item is here already
            m_removed_items.push_back( aItem );
        }
    }

    bool REVISION::IsShadowed( const ITEM* aItem )
    {
        bool here = std::find( m_removed_items.begin(), m_removed_items.end(), aItem )
            != m_removed_items.end();
        return here || (m_parent && m_parent->IsShadowed( aItem ));
    }

    bool REVISION::Owns( const ITEM* aItem ) const
    {
        auto it = std::find_if(
            m_added_items.begin(),
            m_added_items.end(),
            [aItem]( const std::unique_ptr< ITEM >& ptr ) { return ptr.get() == aItem; }
        );

        return it != m_added_items.end();
    }

    std::unique_ptr< REVISION > REVISION::ReleaseBranch( const REVISION* aBranch )
    {
        auto it = std::find_if(
            m_branches.begin(),
            m_branches.end(),
            [aBranch]( const std::unique_ptr< REVISION >& ref ) { return ref.get() == aBranch; }
        );

        if( it != m_branches.end() )
        {
            auto ptr = std::move( *it );
            m_branches.erase( it );
            ptr->m_parent = nullptr;
            ptr->m_depth = 0;
            return ptr;
        }
        else
        {
            return{};
        }
    }

    void REVISION::Absorb( REVISION* aDiff )
    {
        for( auto* item_ptr : aDiff->m_removed_items )
        {
            RemoveItem( item_ptr );
        }
        aDiff->m_removed_items.clear();

        for( auto& item_ptr : aDiff->m_added_items )
        {
            AddItem( std::move( item_ptr ) );
        }
        aDiff->m_added_items.clear();
    }

    REVISION* REVISION::Revert()
    {
        auto result = m_parent;
        m_parent->RemoveBranch( this );
        return result;
    }

    REVISION * REVISION::Squash()
    {
        assert( m_parent );
        m_parent->Absorb( this );

        // Releasing this branch will set parent_ to nullptr, so copy it early.
        auto parent = m_parent;
        auto myself = m_parent->ReleaseBranch( this );

        parent->ClearBranches();

        parent->m_branches.insert( parent->m_branches.end(),
            std::make_move_iterator( m_branches.begin() ),
            std::make_move_iterator( m_branches.end() )
            );

        return parent;
    }

    void REVISION::ClearBranches()
    {
        m_branches.clear();
    }

    REVISION* REVISION::Branch()
    {
        m_branches.emplace_back( std::unique_ptr< REVISION >( new REVISION() ) );
        m_branches.back()->m_parent = this;
        return m_branches.back().get();
    }

    void REVISION::RemoveBranch( REVISION* aBranch )
    {
        ReleaseBranch( aBranch );
    }

    REVISION* REVISION::Parent() const
    {
        return m_parent;
    }

    size_t REVISION::NumChanges() const
    {
        return m_added_items.size() + m_removed_items.size();
    }

    REVISION_PATH REVISION::Path( const REVISION * aAncestor ) const
    {
        std::vector< const REVISION* > result;
        const REVISION* state = this;

        while( state != aAncestor )
        {
            result.push_back( state );
            state = state->Parent();
        }

        return { std::move( result ), {} };
    }

    SEQUENCE< REVISION::ADDED_ITEM_ITERATOR > REVISION::AddedItems()
    {
        return{ m_added_items.begin(), m_added_items.end() };
    }

    SEQUENCE< REVISION::CONST_ADDED_ITEM_ITERATOR > REVISION::AddedItems() const
    {
        return{ m_added_items.begin(), m_added_items.end() };
    }

    SEQUENCE< REVISION::REMOVED_ITEM_ITERATOR> REVISION::RemovedItems()
    {
        return{ m_removed_items.begin(), m_removed_items.end() };
    }

    SEQUENCE< REVISION::CONST_REMOVED_ITEM_ITERATOR> REVISION::RemovedItems() const
    {
        return{ m_removed_items.begin(), m_removed_items.end() };
    }

    SEQUENCE< REVISION::BRANCH_ITERATOR > REVISION::Branches()
    {
        return{ m_branches.begin(), m_branches.end() };
    }

    SEQUENCE< REVISION::CONST_BRANCH_ITERATOR > REVISION::Branches() const
    {
        return{ m_branches.begin(), m_branches.end() };
    }

    REVISION_PATH Path( const REVISION* aFrom, const REVISION* aTo )
    {
        size_t fromDepth = aFrom->Depth();
        size_t toDepth   = aTo->Depth();

        if( aFrom != aTo ) {
            bool break_here = true;
        }

        std::vector< const REVISION* > from, to;
        from.reserve( fromDepth > toDepth   ? fromDepth - toDepth   : 0 );
        to.reserve  ( toDepth   > fromDepth ? toDepth   - fromDepth : 0 );

        while( fromDepth > toDepth ) {
            from.push_back( aFrom );
            aFrom = aFrom->Parent();
            --fromDepth;
        }

        while( toDepth > fromDepth ) {
            to.push_back( aTo );
            aTo = aTo->Parent();
            --toDepth;
        }

        while( aFrom && aTo && aFrom != aTo ) {
            from.push_back( aFrom );
            to.push_back( aTo );
            aFrom = aFrom->Parent();
            aTo = aTo->Parent();
        }

        assert( aFrom == aTo );
        assert( aFrom != nullptr );

        return{ std::move(from), std::move(to) };
    }

    size_t REVISION::Depth() const
    {
        // Could also add this as member, but that would mean more state to keep up to date.
        size_t depth = 0;
        REVISION* rev = m_parent;

        while( rev ) {
            ++depth;
            rev = rev->Parent();
        }

        return depth;
    }

}
