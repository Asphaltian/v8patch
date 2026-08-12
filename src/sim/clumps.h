#pragma once

#include <cstdint>
#include <span>
#include <vector>

namespace v8patch::sim {

class Clumps
{
public:
	void reset(std::int32_t count)
	{
		count_ = count;

		parent_.resize(index(count));
		anchored_.assign(index(count), false);

		for (std::int32_t i = 0; i < count; ++i) {
			parent_[index(i)] = i;
		}
	}

	[[nodiscard]] std::int32_t find(std::int32_t of) noexcept
	{
		while (parent_[index(of)] != of) {
			const std::int32_t above = parent_[index(parent_[index(of)])];

			parent_[index(of)] = above;
			of = above;
		}

		return of;
	}

	void merge(std::int32_t first, std::int32_t second) noexcept
	{
		const std::int32_t a = find(first);
		const std::int32_t b = find(second);

		if (a != b) {
			parent_[index(a)] = b;
			anchored_[index(b)] = anchored_[index(b)] || anchored_[index(a)];
		}
	}

	void anchor(std::int32_t of) noexcept { anchored_[index(find(of))] = true; }

	void drop(std::int32_t of) noexcept { parent_[index(of)] = kDropped; }

	void gather()
	{
		counts_.assign(index(count_), 0);

		for (std::int32_t i = 0; i < count_; ++i) {
			if (!dropped(i)) {
				++counts_[index(find(i))];
			}
		}

		start_.assign(index(count_), 0);
		groups_.clear();

		std::int32_t at = 0;

		for (std::int32_t root = 0; root < count_; ++root) {
			start_[index(root)] = at;

			if (counts_[index(root)] > 0) {
				groups_.push_back(root);
				at += counts_[index(root)];
			}
		}

		members_.resize(index(at));
		cursor_ = start_;

		for (std::int32_t i = 0; i < count_; ++i) {
			if (!dropped(i)) {
				members_[index(cursor_[index(find(i))]++)] = i;
			}
		}
	}

	[[nodiscard]] std::size_t groupCount() const noexcept { return groups_.size(); }

	[[nodiscard]] std::span<const std::int32_t> group(std::size_t at) const noexcept
	{
		const std::int32_t root = groups_[at];

		return {members_.data() + start_[index(root)], index(counts_[index(root)])};
	}

	[[nodiscard]] bool anchored(std::size_t at) const noexcept { return anchored_[index(groups_[at])]; }

private:
	static constexpr std::int32_t kDropped = -1;

	[[nodiscard]] static std::size_t index(std::int32_t of) noexcept { return static_cast<std::size_t>(of); }

	[[nodiscard]] bool dropped(std::int32_t of) const noexcept { return parent_[index(of)] == kDropped; }

	std::vector<std::int32_t> parent_;
	std::vector<std::int32_t> counts_;
	std::vector<std::int32_t> start_;
	std::vector<std::int32_t> cursor_;
	std::vector<std::int32_t> members_;
	std::vector<std::int32_t> groups_;
	std::vector<bool> anchored_;
	std::int32_t count_ = 0;
};

} // namespace v8patch::sim
