#include "sim/clumps.h"

#include <vector>

#include "check.h"

namespace v8patch::tests {
namespace {

using v8patch::sim::Clumps;

std::vector<std::vector<std::int32_t>> groupsOf(Clumps& clumps)
{
	std::vector<std::vector<std::int32_t>> found;

	for (std::size_t at = 0; at < clumps.groupCount(); ++at) {
		const std::span<const std::int32_t> members = clumps.group(at);

		found.emplace_back(members.begin(), members.end());
	}

	return found;
}

void unjoinedPrimitivesEachGetTheirOwnGroup()
{
	Clumps clumps;

	clumps.reset(4);
	clumps.gather();

	const auto found = groupsOf(clumps);

	same(static_cast<int>(found.size()), 4, "four loose primitives make four groups");

	for (std::size_t at = 0; at < found.size(); ++at) {
		same(static_cast<int>(found[at].size()), 1, "each group holds one primitive");
		same(found[at][0], static_cast<std::int32_t>(at), "in ascending order");
	}
}

void rigidJointsMergeIntoOneGroup()
{
	Clumps clumps;

	clumps.reset(6);
	clumps.merge(0, 1);
	clumps.merge(1, 2);
	clumps.merge(4, 5);
	clumps.gather();

	const auto found = groupsOf(clumps);

	same(static_cast<int>(found.size()), 3, "a chain of three, a lone one and a pair");
	same(static_cast<int>(found[0].size()), 3, "the chain holds three");
	same(found[0][0], 0, "members come out in ascending order");
	same(found[0][1], 1, "members come out in ascending order");
	same(found[0][2], 2, "members come out in ascending order");
	same(static_cast<int>(found[1].size()), 1, "the lone primitive is its own group");
	same(found[1][0], 3, "and it is the one that was never merged");
	same(static_cast<int>(found[2].size()), 2, "the pair holds two");
}

void anchoringSurvivesALaterMerge()
{
	for (int order = 0; order < 2; ++order) {
		Clumps clumps;

		clumps.reset(3);

		if (order == 0) {
			clumps.anchor(0);
			clumps.merge(0, 1);
			clumps.merge(1, 2);
		} else {
			clumps.merge(0, 1);
			clumps.anchor(0);
			clumps.merge(1, 2);
		}

		clumps.gather();

		same(static_cast<int>(clumps.groupCount()), 1, "the three merge into one group");
		check(clumps.anchored(0), "and the group is anchored whichever order the anchor arrived in");
	}
}

void unanchoredGroupsStayUnanchored()
{
	Clumps clumps;

	clumps.reset(4);
	clumps.merge(0, 1);
	clumps.anchor(0);
	clumps.merge(2, 3);
	clumps.gather();

	same(static_cast<int>(clumps.groupCount()), 2, "two pairs");
	check(clumps.anchored(0), "the anchored pair is anchored");
	check(!clumps.anchored(1), "the other pair is not");
}

void droppedPrimitivesLeaveNoGroup()
{
	Clumps clumps;

	clumps.reset(4);
	clumps.drop(1);
	clumps.drop(2);
	clumps.gather();

	const auto found = groupsOf(clumps);

	same(static_cast<int>(found.size()), 2, "only the two with bodies group");
	same(found[0][0], 0, "the first survivor");
	same(found[1][0], 3, "the last survivor");
}

void mergeOrderDoesNotChangeMembership()
{
	Clumps forward;
	Clumps backward;

	forward.reset(5);
	forward.merge(0, 1);
	forward.merge(1, 2);
	forward.merge(3, 4);
	forward.gather();

	backward.reset(5);
	backward.merge(3, 4);
	backward.merge(2, 1);
	backward.merge(1, 0);
	backward.gather();

	const auto a = groupsOf(forward);
	const auto b = groupsOf(backward);

	same(static_cast<int>(a.size()), static_cast<int>(b.size()), "the same number of groups either way");

	for (std::size_t at = 0; at < a.size(); ++at) {
		check(a[at] == b[at], "holding the same members in the same order");
	}
}

void resetClearsTheLastPass()
{
	Clumps clumps;

	clumps.reset(3);
	clumps.merge(0, 1);
	clumps.anchor(0);
	clumps.gather();

	same(static_cast<int>(clumps.groupCount()), 2, "a pair and a single");

	clumps.reset(3);
	clumps.gather();

	same(static_cast<int>(clumps.groupCount()), 3, "and nothing carries into the next pass");
	check(!clumps.anchored(0), "including the anchor");
}

} // namespace

void runClumps()
{
	unjoinedPrimitivesEachGetTheirOwnGroup();
	rigidJointsMergeIntoOneGroup();
	anchoringSurvivesALaterMerge();
	unanchoredGroupsStayUnanchored();
	droppedPrimitivesLeaveNoGroup();
	mergeOrderDoesNotChangeMembership();
	resetClearsTheLastPass();
}

} // namespace v8patch::tests
