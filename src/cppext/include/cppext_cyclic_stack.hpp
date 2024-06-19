#ifndef CPPEXT_CYCLIC_STACK_INCLUDED
#define CPPEXT_CYCLIC_STACK_INCLUDED

#include <cppext_numeric.hpp>

#include <algorithm>
#include <span>
#include <vector>

namespace cppext
{
    template<typename T, typename Container = std::vector<T>>
    struct [[nodiscard]] cyclic_stack final
    {
    public:
        using container_type = Container;
        using value_type = typename Container::value_type;
        using size_type = typename Container::size_type;
        using reference = typename Container::reference;
        using const_reference = typename Container::const_reference;

    public:
        cyclic_stack() = default;

        explicit cyclic_stack(size_type cycle, size_type size = {});

        cyclic_stack(cyclic_stack const&) = default;

        cyclic_stack(cyclic_stack&&) noexcept = default;

    public:
        ~cyclic_stack() = default;

    public:
        [[nodiscard]] T& top();

        [[nodiscard]] T const& top() const;

        [[nodiscard]] bool empty() const;

        [[nodiscard]] size_type index() const;

        void cycle();

        void push(value_type const& value);

        void push(value_type&& value);

        template<class... Args>
        decltype(auto) emplace(Args&&... args);

        void pop();

        void swap(cyclic_stack& other) noexcept;

        [[nodiscard]] T* data();

        [[nodiscard]] T const* data() const;

        // cppcheck-suppress functionConst
        [[nodiscard]] std::span<T> as_span();

        [[nodiscard]] std::span<T const> as_span() const;

    public:
        cyclic_stack& operator=(cyclic_stack const&) = default;

        cyclic_stack& operator=(cyclic_stack&&) noexcept = default;

    private:
        Container data_;
        Container::size_type cycle_{};
        Container::size_type index_{};
    };

    template<typename T, typename Container>
    cyclic_stack<T, Container>::cyclic_stack(size_type const cycle,
        size_type const size)
        : cycle_{cycle}
    {
        data_.resize(size);
    }

    template<typename T, typename Container>
    T& cyclic_stack<T, Container>::top()
    {
        return data_[index_];
    }

    template<typename T, typename Container>
    T const& cyclic_stack<T, Container>::top() const
    {
        return data_[index_];
    }

    template<typename T, typename Container>
    bool cyclic_stack<T, Container>::empty() const
    {
        return data_.empty();
    }

    template<typename T, typename Container>
    cyclic_stack<T, Container>::size_type
    cyclic_stack<T, Container>::index() const
    {
        return index_;
    }

    template<typename T, typename Container>
    void cyclic_stack<T, Container>::cycle()
    {
        index_ = (index_ + 1) % cycle_;
    }

    template<typename T, typename Container>
    void cyclic_stack<T, Container>::push(value_type const& value)
    {
        data_.push_back(value);
    }

    template<typename T, typename Container>
    void cyclic_stack<T, Container>::push(value_type&& value)
    {
        data_.push_back(std::move(value));
    }

    template<typename T, typename Container>
    template<class... Args>
    decltype(auto) cyclic_stack<T, Container>::emplace(Args&&... args)
    {
        return data_.emplace_back(std::forward<Args>(args)...);
    }

    template<typename T, typename Container>
    void cyclic_stack<T, Container>::pop()
    {
        auto const current_it{std::next(begin(data_),
            cppext::narrow<typename Container::difference_type>(index_))};
        std::move(std::next(current_it, 1), end(data_), current_it);
    }

    template<typename T, typename Container>
    void cyclic_stack<T, Container>::swap(cyclic_stack& other) noexcept
    {
        using std::swap;
        swap(data_, other.data_);
        swap(cycle_, other.cycle_);
        swap(index_, other.index_);
    }

    template<typename T, typename Container>
    T* cyclic_stack<T, Container>::data()
    {
        return data_.data();
    }

    template<typename T, typename Container>
    T const* cyclic_stack<T, Container>::data() const
    {
        return data_.data();
    }

    template<typename T, typename Container>
    std::span<T> cyclic_stack<T, Container>::as_span()
    {
        return data_;
    }

    template<typename T, typename Container>
    std::span<T const> cyclic_stack<T, Container>::as_span() const
    {
        return data_;
    }
} // namespace cppext

#endif
