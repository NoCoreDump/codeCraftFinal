#ifndef TSL_ROBIN_MAP_H
#define TSL_ROBIN_MAP_H
#ifndef TSL_ROBIN_HASH_H
#define TSL_ROBIN_HASH_H
#ifndef TSL_ROBIN_GROWTH_POLICY_H
#define TSL_ROBIN_GROWTH_POLICY_H

#include <bits/stdc++.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <mutex>
#include <cstddef>
#include <initializer_list>
#include <type_traits>
#include <utility>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <vector>
#include <array>
#include <climits>
#include <ratio>
#ifdef TSL_DEBUG
#else
#    define tsl_rh_assert(expr) (static_cast<void>(0))
#endif
#if (defined(__cpp_exceptions) || defined(__EXCEPTIONS) || (defined (_MSC_VER) && defined (_CPPUNWIND))) && !defined(TSL_NO_EXCEPTIONS)
#    define TSL_RH_THROW_OR_TERMINATE(ex, msg) throw ex(msg)
#endif
#if defined(__GNUC__) || defined(__clang__)
#    define TSL_RH_LIKELY(exp) (__builtin_expect(!!(exp), true))
#else
#endif
namespace tsl {
    namespace rh {

/**
 * Grow the hash table by a factor of GrowthFactor keeping the bucket count to a power of two. It allows
 * the table to use a mask operation instead of a modulo operation to map a hash to a bucket.
 *
 * GrowthFactor must be a power of two >= 2.
 */
        template<std::size_t GrowthFactor>
        class power_of_two_growth_policy {
        public:
            /**
             * Called on the hash table creation and on rehash. The number of buckets for the table is passed in parameter.
             * This number is a minimum, the policy may update this value with a higher value if needed (but not lower).
             *
             * If 0 is given, min_bucket_count_in_out must still be 0 after the policy creation and
             * bucket_for_hash must always return 0 in this case.
             */
            explicit power_of_two_growth_policy(std::size_t &min_bucket_count_in_out) {
                if (min_bucket_count_in_out > max_bucket_count()) {
                    TSL_RH_THROW_OR_TERMINATE(std::length_error, "The hash table exceeds its maxmimum size.");
                }

                if (min_bucket_count_in_out > 0) {
                    min_bucket_count_in_out = round_up_to_power_of_two(min_bucket_count_in_out);
                    m_mask = min_bucket_count_in_out - 1;
                } else {
                    m_mask = 0;
                }
            }

            /**
             * Return the bucket [0, bucket_count()) to which the hash belongs.
             * If bucket_count() is 0, it must always return 0.
             */
            std::size_t bucket_for_hash(std::size_t hash) const noexcept {
                return hash & m_mask;
            }

            /**
             * Return the number of buckets that should be used on next growth.
             */
            std::size_t next_bucket_count() const {
                if ((m_mask + 1) > max_bucket_count() / GrowthFactor) {
                    TSL_RH_THROW_OR_TERMINATE(std::length_error, "The hash table exceeds its maxmimum size.");
                }

                return (m_mask + 1) * GrowthFactor;
            }

            /**
             * Return the maximum number of buckets supported by the policy.
             */
            std::size_t max_bucket_count() const {
                // Largest power of two.
                return (std::numeric_limits<std::size_t>::max() / 2) + 1;
            }

            /**
             * Reset the growth policy as if it was created with a bucket count of 0.
             * After a clear, the policy must always return 0 when bucket_for_hash is called.
             */
            void clear() noexcept {
                m_mask = 0;
            }

        private:
            static std::size_t round_up_to_power_of_two(std::size_t value) {
                if (is_power_of_two(value)) {
                    return value;
                }

                if (value == 0) {
                    return 1;
                }

                --value;
                for (std::size_t i = 1; i < sizeof(std::size_t) * CHAR_BIT; i *= 2) {
                    value |= value >> i;
                }

                return value + 1;
            }

            static constexpr bool is_power_of_two(std::size_t value) {
                return value != 0 && (value & (value - 1)) == 0;
            }

        protected:
            static_assert(is_power_of_two(GrowthFactor) && GrowthFactor >= 2,
                          "GrowthFactor must be a power of two >= 2.");

            std::size_t m_mask;
        };


/**
 * Grow the hash table by GrowthFactor::num / GrowthFactor::den and use a modulo to map a hash
 * to a bucket. Slower but it can be useful if you want a slower growth.
 */
        template<class GrowthFactor = std::ratio<3, 2>>
        class mod_growth_policy {
        public:
            explicit mod_growth_policy(std::size_t &min_bucket_count_in_out) {
                if (min_bucket_count_in_out > max_bucket_count()) {
                    TSL_RH_THROW_OR_TERMINATE(std::length_error, "The hash table exceeds its maxmimum size.");
                }

                if (min_bucket_count_in_out > 0) {
                    m_mod = min_bucket_count_in_out;
                } else {
                    m_mod = 1;
                }
            }

            std::size_t bucket_for_hash(std::size_t hash) const noexcept {
                return hash % m_mod;
            }

            std::size_t next_bucket_count() const {
                if (m_mod == max_bucket_count()) {
                    TSL_RH_THROW_OR_TERMINATE(std::length_error, "The hash table exceeds its maxmimum size.");
                }

                const double next_bucket_count = std::ceil(double(m_mod) * REHASH_SIZE_MULTIPLICATION_FACTOR);
                if (!std::isnormal(next_bucket_count)) {
                    TSL_RH_THROW_OR_TERMINATE(std::length_error, "The hash table exceeds its maxmimum size.");
                }

                if (next_bucket_count > double(max_bucket_count())) {
                    return max_bucket_count();
                } else {
                    return std::size_t(next_bucket_count);
                }
            }

            std::size_t max_bucket_count() const {
                return MAX_BUCKET_COUNT;
            }

            void clear() noexcept {
                m_mod = 1;
            }

        private:
            static constexpr double REHASH_SIZE_MULTIPLICATION_FACTOR = 1.0 * GrowthFactor::num / GrowthFactor::den;
            static const std::size_t MAX_BUCKET_COUNT =
                    std::size_t(double(
                            std::numeric_limits<std::size_t>::max() / REHASH_SIZE_MULTIPLICATION_FACTOR
                    ));

            static_assert(REHASH_SIZE_MULTIPLICATION_FACTOR >= 1.1, "Growth factor should be >= 1.1.");

            std::size_t m_mod;
        };


        namespace detail {

#if SIZE_MAX >= ULLONG_MAX
#define TSL_RH_NB_PRIMES 51
#elif SIZE_MAX >= ULONG_MAX
            #define TSL_RH_NB_PRIMES 40
#else
#define TSL_RH_NB_PRIMES 23
#endif

            static constexpr const std::array<std::size_t, TSL_RH_NB_PRIMES> PRIMES = {{
                                                                                               1u, 5u, 17u, 29u, 37u, 53u, 67u, 79u, 97u, 131u, 193u, 257u, 389u, 521u, 769u, 1031u,
                                                                                               1543u, 2053u, 3079u, 6151u, 12289u, 24593u, 49157u,
#if SIZE_MAX >= ULONG_MAX
                                                                                               98317ul, 196613ul, 393241ul, 786433ul, 1572869ul, 3145739ul, 6291469ul, 12582917ul,
                                                                                               25165843ul, 50331653ul, 100663319ul, 201326611ul, 402653189ul, 805306457ul, 1610612741ul,
                                                                                               3221225473ul, 4294967291ul,
#endif
#if SIZE_MAX >= ULLONG_MAX
                                                                                               6442450939ull, 12884901893ull, 25769803751ull, 51539607551ull, 103079215111ull, 206158430209ull,
                                                                                               412316860441ull, 824633720831ull, 1649267441651ull, 3298534883309ull, 6597069766657ull,
#endif
                                                                                       }};

            template<unsigned int IPrime>
            static constexpr std::size_t mod(std::size_t hash) { return hash % PRIMES[IPrime]; }

// MOD_PRIME[iprime](hash) returns hash % PRIMES[iprime]. This table allows for faster modulo as the
// compiler can optimize the modulo code better with a constant known at the compilation.
            static constexpr const std::array<std::size_t(*)(std::size_t), TSL_RH_NB_PRIMES> MOD_PRIME = {{
                                                                                                                  &mod<0>, &mod<1>, &mod<2>, &mod<3>, &mod<4>, &mod<5>, &mod<6>, &mod<7>, &mod<8>, &mod<9>, &mod<10>,
                                                                                                                  &mod<11>, &mod<12>, &mod<13>, &mod<14>, &mod<15>, &mod<16>, &mod<17>, &mod<18>, &mod<19>, &mod<20>,
                                                                                                                  &mod<21>, &mod<22>,
#if SIZE_MAX >= ULONG_MAX
                                                                                                                  &mod<23>, &mod<24>, &mod<25>, &mod<26>, &mod<27>, &mod<28>, &mod<29>, &mod<30>, &mod<31>, &mod<32>,
                                                                                                                  &mod<33>, &mod<34>, &mod<35>, &mod<36>, &mod<37>, &mod<38>, &mod<39>,
#endif
#if SIZE_MAX >= ULLONG_MAX
                                                                                                                  &mod<40>, &mod<41>, &mod<42>, &mod<43>, &mod<44>, &mod<45>, &mod<46>, &mod<47>, &mod<48>, &mod<49>,
                                                                                                                  &mod<50>,
#endif
                                                                                                          }};

        }

/**
 * Grow the hash table by using prime numbers as bucket count. Slower than tsl::rh::power_of_two_growth_policy in
 * general but will probably distribute the values around better in the buckets with a poor hash function.
 *
 * To allow the compiler to optimize the modulo operation, a lookup table is used with constant primes numbers.
 *
 * With a switch the code would look like:
 * \code
 * switch(iprime) { // iprime is the current prime of the hash table
 *     case 0: hash % 5ul;
 *             break;
 *     case 1: hash % 17ul;
 *             break;
 *     case 2: hash % 29ul;
 *             break;
 *     ...
 * }
 * \endcode
 *
 * Due to the constant variable in the modulo the compiler is able to optimize the operation
 * by a series of multiplications, substractions and shifts.
 *
 * The 'hash % 5' could become something like 'hash - (hash * 0xCCCCCCCD) >> 34) * 5' in a 64 bits environement.
 */
        class prime_growth_policy {
        public:
            explicit prime_growth_policy(std::size_t &min_bucket_count_in_out) {
                auto it_prime = std::lower_bound(detail::PRIMES.begin(),
                                                 detail::PRIMES.end(), min_bucket_count_in_out);
                if (it_prime == detail::PRIMES.end()) {
                    TSL_RH_THROW_OR_TERMINATE(std::length_error, "The hash table exceeds its maxmimum size.");
                }

                m_iprime = static_cast<unsigned int>(std::distance(detail::PRIMES.begin(), it_prime));
                if (min_bucket_count_in_out > 0) {
                    min_bucket_count_in_out = *it_prime;
                } else {
                    min_bucket_count_in_out = 0;
                }
            }

            std::size_t bucket_for_hash(std::size_t hash) const noexcept {
                return detail::MOD_PRIME[m_iprime](hash);
            }

            std::size_t next_bucket_count() const {
                if (m_iprime + 1 >= detail::PRIMES.size()) {
                    TSL_RH_THROW_OR_TERMINATE(std::length_error, "The hash table exceeds its maxmimum size.");
                }

                return detail::PRIMES[m_iprime + 1];
            }

            std::size_t max_bucket_count() const {
                return detail::PRIMES.back();
            }

            void clear() noexcept {
                m_iprime = 0;
            }

        private:
            unsigned int m_iprime;

            static_assert(std::numeric_limits<decltype(m_iprime)>::max() >= detail::PRIMES.size(),
                          "The type of m_iprime is not big enough.");
        };

    }
}
#endif
namespace tsl {

    namespace detail_robin_hash {

        template<typename T>
        struct make_void {
            using type = void;
        };

        template<typename T, typename = void>
        struct has_is_transparent : std::false_type {
        };

        template<typename T>
        struct has_is_transparent<T, typename make_void<typename T::is_transparent>::type> : std::true_type {
        };

        template<typename U>
        struct is_power_of_two_policy : std::false_type {
        };

        template<std::size_t GrowthFactor>
        struct is_power_of_two_policy<tsl::rh::power_of_two_growth_policy<GrowthFactor>> : std::true_type {
        };

// Only available in C++17, we need to be compatible with C++11
        template<class T>
        const T &clamp(const T &v, const T &lo, const T &hi) {
            return std::min(hi, std::max(lo, v));
        }

        template<typename T, typename U>
        static T numeric_cast(U value, const char *error_message = "numeric_cast() failed.") {
            T ret = static_cast<T>(value);
            if (static_cast<U>(ret) != value) {
                TSL_RH_THROW_OR_TERMINATE(std::runtime_error, error_message);
            }

            const bool is_same_signedness = (std::is_unsigned<T>::value && std::is_unsigned<U>::value) ||
                                            (std::is_signed<T>::value && std::is_signed<U>::value);
            if (!is_same_signedness && (ret < T{}) != (value < U{})) {
                TSL_RH_THROW_OR_TERMINATE(std::runtime_error, error_message);
            }

            return ret;
        }


        using truncated_hash_type = std::uint_least32_t;

/**
 * Helper class that stores a truncated hash if StoreHash is true and nothing otherwise.
 */
        template<bool StoreHash>
        class bucket_entry_hash {
        public:
            bool bucket_hash_equal(std::size_t /*hash*/) const noexcept {
                return true;
            }

            truncated_hash_type truncated_hash() const noexcept {
                return 0;
            }

        protected:
            void set_hash(truncated_hash_type /*hash*/) noexcept {
            }
        };

        template<>
        class bucket_entry_hash<true> {
        public:
            bool bucket_hash_equal(std::size_t hash) const noexcept {
                return m_hash == truncated_hash_type(hash);
            }

            truncated_hash_type truncated_hash() const noexcept {
                return m_hash;
            }

        protected:
            void set_hash(truncated_hash_type hash) noexcept {
                m_hash = truncated_hash_type(hash);
            }

        private:
            truncated_hash_type m_hash;
        };


/**
 * Each bucket entry has:
 * - A value of type `ValueType`.
 * - An integer to store how far the value of the bucket, if any, is from its ideal bucket
 *   (ex: if the current bucket 5 has the value 'foo' and `hash('foo') % nb_buckets` == 3,
 *        `dist_from_ideal_bucket()` will return 2 as the current value of the bucket is two
 *        buckets away from its ideal bucket)
 *   If there is no value in the bucket (i.e. `empty()` is true) `dist_from_ideal_bucket()` will be < 0.
 * - A marker which tells us if the bucket is the last bucket of the bucket array (useful for the
 *   iterator of the hash table).
 * - If `StoreHash` is true, 32 bits of the hash of the value, if any, are also stored in the bucket.
 *   If the size of the hash is more than 32 bits, it is truncated. We don't store the full hash
 *   as storing the hash is a potential opportunity to use the unused space due to the alignement
 *   of the bucket_entry structure. We can thus potentially store the hash without any extra space
 *   (which would not be possible with 64 bits of the hash).
 */
        template<typename ValueType, bool StoreHash>
        class bucket_entry : public bucket_entry_hash<StoreHash> {
            using bucket_hash = bucket_entry_hash<StoreHash>;

        public:
            using value_type = ValueType;
            using distance_type = std::int_least16_t;


            bucket_entry() noexcept: bucket_hash(), m_dist_from_ideal_bucket(EMPTY_MARKER_DIST_FROM_IDEAL_BUCKET),
                                     m_last_bucket(false) {
                tsl_rh_assert(empty());
            }

            bucket_entry(bool last_bucket) noexcept
                    : bucket_hash(), m_dist_from_ideal_bucket(EMPTY_MARKER_DIST_FROM_IDEAL_BUCKET),
                      m_last_bucket(last_bucket) {
                tsl_rh_assert(empty());
            }

            bucket_entry(const bucket_entry &other) noexcept(std::is_nothrow_copy_constructible<value_type>::value):
                    bucket_hash(other),
                    m_dist_from_ideal_bucket(EMPTY_MARKER_DIST_FROM_IDEAL_BUCKET),
                    m_last_bucket(other.m_last_bucket) {
                if (!other.empty()) {
                    ::new(static_cast<void *>(std::addressof(m_value))) value_type(other.value());
                    m_dist_from_ideal_bucket = other.m_dist_from_ideal_bucket;
                }
            }

            /**
             * Never really used, but still necessary as we must call resize on an empty `std::vector<bucket_entry>`.
             * and we need to support move-only types. See robin_hash constructor for details.
             */
            bucket_entry(bucket_entry &&other) noexcept(std::is_nothrow_move_constructible<value_type>::value):
                    bucket_hash(std::move(other)),
                    m_dist_from_ideal_bucket(EMPTY_MARKER_DIST_FROM_IDEAL_BUCKET),
                    m_last_bucket(other.m_last_bucket) {
                if (!other.empty()) {
                    ::new(static_cast<void *>(std::addressof(m_value))) value_type(std::move(other.value()));
                    m_dist_from_ideal_bucket = other.m_dist_from_ideal_bucket;
                }
            }

            bucket_entry &operator=(const bucket_entry &other)
            noexcept(std::is_nothrow_copy_constructible<value_type>::value) {
                if (this != &other) {
                    clear();

                    bucket_hash::operator=(other);
                    if (!other.empty()) {
                        ::new(static_cast<void *>(std::addressof(m_value))) value_type(other.value());
                    }

                    m_dist_from_ideal_bucket = other.m_dist_from_ideal_bucket;
                    m_last_bucket = other.m_last_bucket;
                }

                return *this;
            }

            bucket_entry &operator=(bucket_entry &&) = delete;

            ~bucket_entry() noexcept {
                clear();
            }

            void clear() noexcept {
                if (!empty()) {
                    destroy_value();
                    m_dist_from_ideal_bucket = EMPTY_MARKER_DIST_FROM_IDEAL_BUCKET;
                }
            }

            bool empty() const noexcept {
                return m_dist_from_ideal_bucket == EMPTY_MARKER_DIST_FROM_IDEAL_BUCKET;
            }

            value_type &value() noexcept {
                tsl_rh_assert(!empty());
                return *reinterpret_cast<value_type *>(std::addressof(m_value));
            }

            const value_type &value() const noexcept {
                tsl_rh_assert(!empty());
                return *reinterpret_cast<const value_type *>(std::addressof(m_value));
            }

            distance_type dist_from_ideal_bucket() const noexcept {
                return m_dist_from_ideal_bucket;
            }

            bool last_bucket() const noexcept {
                return m_last_bucket;
            }

            void set_as_last_bucket() noexcept {
                m_last_bucket = true;
            }

            template<typename... Args>
            void set_value_of_empty_bucket(distance_type dist_from_ideal_bucket,
                                           truncated_hash_type hash, Args &&... value_type_args) {
                tsl_rh_assert(dist_from_ideal_bucket >= 0);
                tsl_rh_assert(empty());

                ::new(static_cast<void *>(std::addressof(m_value))) value_type(std::forward<Args>(value_type_args)...);
                this->set_hash(hash);
                m_dist_from_ideal_bucket = dist_from_ideal_bucket;

                tsl_rh_assert(!empty());
            }

            void swap_with_value_in_bucket(distance_type &dist_from_ideal_bucket,
                                           truncated_hash_type &hash, value_type &value) {
                tsl_rh_assert(!empty());

                using std::swap;
                swap(value, this->value());
                swap(dist_from_ideal_bucket, m_dist_from_ideal_bucket);

                // Avoid warning of unused variable if StoreHash is false
                (void) hash;
                if (StoreHash) {
                    const truncated_hash_type tmp_hash = this->truncated_hash();
                    this->set_hash(hash);
                    hash = tmp_hash;
                }
            }

            static truncated_hash_type truncate_hash(std::size_t hash) noexcept {
                return truncated_hash_type(hash);
            }

        private:
            void destroy_value() noexcept {
                tsl_rh_assert(!empty());
                value().~value_type();
            }

        public:
            static const distance_type DIST_FROM_IDEAL_BUCKET_LIMIT = 4096;
            static_assert(DIST_FROM_IDEAL_BUCKET_LIMIT <= std::numeric_limits<distance_type>::max() - 1,
                          "DIST_FROM_IDEAL_BUCKET_LIMIT must be <= std::numeric_limits<distance_type>::max() - 1.");

        private:
            using storage = typename std::aligned_storage<sizeof(value_type), alignof(value_type)>::type;

            static const distance_type EMPTY_MARKER_DIST_FROM_IDEAL_BUCKET = -1;

            distance_type m_dist_from_ideal_bucket;
            bool m_last_bucket;
            storage m_value;
        };


/**
 * Internal common class used by `robin_map` and `robin_set`.
 *
 * ValueType is what will be stored by `robin_hash` (usually `std::pair<Key, T>` for map and `Key` for set).
 *
 * `KeySelect` should be a `FunctionObject` which takes a `ValueType` in parameter and returns a
 *  reference to the key.
 *
 * `ValueSelect` should be a `FunctionObject` which takes a `ValueType` in parameter and returns a
 *  reference to the value. `ValueSelect` should be void if there is no value (in a set for example).
 *
 * The strong exception guarantee only holds if the expression
 * `std::is_nothrow_swappable<ValueType>::value && std::is_nothrow_move_constructible<ValueType>::value` is true.
 *
 * Behaviour is undefined if the destructor of `ValueType` throws.
 */
        template<class ValueType,
                class KeySelect,
                class ValueSelect,
                class Hash,
                class KeyEqual,
                class Allocator,
                bool StoreHash,
                class GrowthPolicy>
        class robin_hash : private Hash, private KeyEqual, private GrowthPolicy {
        private:
            template<typename U>
            using has_mapped_type = typename std::integral_constant<bool, !std::is_same<U, void>::value>;

            static_assert(noexcept(std::declval<GrowthPolicy>().bucket_for_hash(std::size_t(0))),
                          "GrowthPolicy::bucket_for_hash must be noexcept.");
            static_assert(noexcept(std::declval<GrowthPolicy>().clear()), "GrowthPolicy::clear must be noexcept.");

        public:
            template<bool IsConst>
            class robin_iterator;

            using key_type = typename KeySelect::key_type;
            using value_type = ValueType;
            using size_type = std::size_t;
            using difference_type = std::ptrdiff_t;
            using hasher = Hash;
            using key_equal = KeyEqual;
            using allocator_type = Allocator;
            using reference = value_type &;
            using const_reference = const value_type &;
            using pointer = value_type *;
            using const_pointer = const value_type *;
            using iterator = robin_iterator<false>;
            using const_iterator = robin_iterator<true>;


        private:
            /**
             * Either store the hash because we are asked by the `StoreHash` template parameter
             * or store the hash because it doesn't cost us anything in size and can be used to speed up rehash.
             */
            static constexpr bool STORE_HASH = StoreHash ||
                                               (
                                                       (sizeof(tsl::detail_robin_hash::bucket_entry<value_type, true>) ==
                                                        sizeof(tsl::detail_robin_hash::bucket_entry<value_type, false>))
                                                       &&
                                                       (sizeof(std::size_t) == sizeof(truncated_hash_type) ||
                                                        is_power_of_two_policy<GrowthPolicy>::value)
                                                       &&
                                                       // Don't store the hash for primitive types with default hash.
                                                       (!std::is_arithmetic<key_type>::value ||
                                                        !std::is_same<Hash, std::hash<key_type>>::value)
                                               );

            /**
             * Only use the stored hash on lookup if we are explictly asked. We are not sure how slow
             * the KeyEqual operation is. An extra comparison may slow things down with a fast KeyEqual.
             */
            static constexpr bool USE_STORED_HASH_ON_LOOKUP = StoreHash;

            /**
             * We can only use the hash on rehash if the size of the hash type is the same as the stored one or
             * if we use a power of two modulo. In the case of the power of two modulo, we just mask
             * the least significant bytes, we just have to check that the truncated_hash_type didn't truncated
             * more bytes.
             */
            static bool USE_STORED_HASH_ON_REHASH(size_type bucket_count) {
                (void) bucket_count;
                if (STORE_HASH && sizeof(std::size_t) == sizeof(truncated_hash_type)) {
                    return true;
                } else if (STORE_HASH && is_power_of_two_policy<GrowthPolicy>::value) {
                    tsl_rh_assert(bucket_count > 0);
                    return (bucket_count - 1) <= std::numeric_limits<truncated_hash_type>::max();
                } else {
                    return false;
                }
            }

            using bucket_entry = tsl::detail_robin_hash::bucket_entry<value_type, STORE_HASH>;
            using distance_type = typename bucket_entry::distance_type;

            using buckets_allocator = typename std::allocator_traits<allocator_type>::template rebind_alloc<bucket_entry>;
            using buckets_container_type = std::vector<bucket_entry, buckets_allocator>;


        public:
            /**
             * The 'operator*()' and 'operator->()' methods return a const reference and const pointer respectively to the
             * stored value type.
             *
             * In case of a map, to get a mutable reference to the value associated to a key (the '.second' in the
             * stored pair), you have to call 'value()'.
             *
             * The main reason for this is that if we returned a `std::pair<Key, T>&` instead
             * of a `const std::pair<Key, T>&`, the user may modify the key which will put the map in a undefined state.
             */
            template<bool IsConst>
            class robin_iterator {
                friend class robin_hash;

            private:
                using bucket_entry_ptr = typename std::conditional<IsConst,
                        const bucket_entry *,
                        bucket_entry *>::type;


                robin_iterator(bucket_entry_ptr bucket) noexcept: m_bucket(bucket) {
                }

            public:
                using iterator_category = std::forward_iterator_tag;
                using value_type = const typename robin_hash::value_type;
                using difference_type = std::ptrdiff_t;
                using reference = value_type &;
                using pointer = value_type *;


                robin_iterator() noexcept {
                }

                // Copy constructor from iterator to const_iterator.
                template<bool TIsConst = IsConst, typename std::enable_if<TIsConst>::type * = nullptr>
                robin_iterator(const robin_iterator<!TIsConst> &other) noexcept: m_bucket(other.m_bucket) {
                }

                robin_iterator(const robin_iterator &other) = default;

                robin_iterator(robin_iterator &&other) = default;

                robin_iterator &operator=(const robin_iterator &other) = default;

                robin_iterator &operator=(robin_iterator &&other) = default;

                const typename robin_hash::key_type &key() const {
                    return KeySelect()(m_bucket->value());
                }

                template<class U = ValueSelect, typename std::enable_if<
                        has_mapped_type<U>::value && IsConst>::type * = nullptr>
                const typename U::value_type &value() const {
                    return U()(m_bucket->value());
                }

                template<class U = ValueSelect, typename std::enable_if<
                        has_mapped_type<U>::value && !IsConst>::type * = nullptr>
                typename U::value_type &value() {
                    return U()(m_bucket->value());
                }

                reference operator*() const {
                    return m_bucket->value();
                }

                pointer operator->() const {
                    return std::addressof(m_bucket->value());
                }

                robin_iterator &operator++() {
                    while (true) {
                        if (m_bucket->last_bucket()) {
                            ++m_bucket;
                            return *this;
                        }

                        ++m_bucket;
                        if (!m_bucket->empty()) {
                            return *this;
                        }
                    }
                }

                robin_iterator operator++(int) {
                    robin_iterator tmp(*this);
                    ++*this;

                    return tmp;
                }

                friend bool operator==(const robin_iterator &lhs, const robin_iterator &rhs) {
                    return lhs.m_bucket == rhs.m_bucket;
                }

                friend bool operator!=(const robin_iterator &lhs, const robin_iterator &rhs) {
                    return !(lhs == rhs);
                }

            private:
                bucket_entry_ptr m_bucket;
            };


        public:
#if defined(__cplusplus) && __cplusplus >= 201402L

            robin_hash(size_type bucket_count,
                       const Hash &hash,
                       const KeyEqual &equal,
                       const Allocator &alloc,
                       float min_load_factor = DEFAULT_MIN_LOAD_FACTOR,
                       float max_load_factor = DEFAULT_MAX_LOAD_FACTOR) :
                    Hash(hash),
                    KeyEqual(equal),
                    GrowthPolicy(bucket_count),
                    m_buckets_data(
                            [&]() {
                                if (bucket_count > max_bucket_count()) {
                                    TSL_RH_THROW_OR_TERMINATE(std::length_error,
                                                              "The map exceeds its maximum bucket count.");
                                }

                                return bucket_count;
                            }(), alloc
                    ),
                    m_buckets(m_buckets_data.empty() ? static_empty_bucket_ptr() : m_buckets_data.data()),
                    m_bucket_count(bucket_count),
                    m_nb_elements(0),
                    m_grow_on_next_insert(false),
                    m_try_skrink_on_next_insert(false) {
                if (m_bucket_count > 0) {
                    tsl_rh_assert(!m_buckets_data.empty());
                    m_buckets_data.back().set_as_last_bucket();
                }

                this->min_load_factor(min_load_factor);
                this->max_load_factor(max_load_factor);
            }

#else
            /**
             * C++11 doesn't support the creation of a std::vector with a custom allocator and 'count' default-inserted elements.
             * The needed contructor `explicit vector(size_type count, const Allocator& alloc = Allocator());` is only
             * available in C++14 and later. We thus must resize after using the `vector(const Allocator& alloc)` constructor.
             *
             * We can't use `vector(size_type count, const T& value, const Allocator& alloc)` as it requires the
             * value T to be copyable.
             */
            robin_hash(size_type bucket_count,
                       const Hash& hash,
                       const KeyEqual& equal,
                       const Allocator& alloc,
                       float min_load_factor = DEFAULT_MIN_LOAD_FACTOR,
                       float max_load_factor = DEFAULT_MAX_LOAD_FACTOR):
                    Hash(hash),
                    KeyEqual(equal),
                    GrowthPolicy(bucket_count),
                    m_buckets_data(alloc),
                    m_buckets(static_empty_bucket_ptr()),
                    m_bucket_count(bucket_count),
                    m_nb_elements(0),
                    m_grow_on_next_insert(false),
                    m_try_skrink_on_next_insert(false)
            {
                if(bucket_count > max_bucket_count()) {
                    TSL_RH_THROW_OR_TERMINATE(std::length_error, "The map exceeds its maxmimum bucket count.");
                }

                if(m_bucket_count > 0) {
                    m_buckets_data.resize(m_bucket_count);
                    m_buckets = m_buckets_data.data();

                    tsl_rh_assert(!m_buckets_data.empty());
                    m_buckets_data.back().set_as_last_bucket();
                }

                this->min_load_factor(min_load_factor);
                this->max_load_factor(max_load_factor);
            }
#endif

            robin_hash(const robin_hash &other) : Hash(other),
                                                  KeyEqual(other),
                                                  GrowthPolicy(other),
                                                  m_buckets_data(other.m_buckets_data),
                                                  m_buckets(m_buckets_data.empty() ? static_empty_bucket_ptr()
                                                                                   : m_buckets_data.data()),
                                                  m_bucket_count(other.m_bucket_count),
                                                  m_nb_elements(other.m_nb_elements),
                                                  m_load_threshold(other.m_load_threshold),
                                                  m_min_load_factor(other.m_min_load_factor),
                                                  m_max_load_factor(other.m_max_load_factor),
                                                  m_grow_on_next_insert(other.m_grow_on_next_insert),
                                                  m_try_skrink_on_next_insert(other.m_try_skrink_on_next_insert) {
            }

            robin_hash(robin_hash &&other) noexcept(std::is_nothrow_move_constructible<Hash>::value &&
                                                    std::is_nothrow_move_constructible<KeyEqual>::value &&
                                                    std::is_nothrow_move_constructible<GrowthPolicy>::value &&
                                                    std::is_nothrow_move_constructible<buckets_container_type>::value)
                    : Hash(std::move(static_cast<Hash &>(other))),
                      KeyEqual(std::move(static_cast<KeyEqual &>(other))),
                      GrowthPolicy(std::move(static_cast<GrowthPolicy &>(other))),
                      m_buckets_data(std::move(other.m_buckets_data)),
                      m_buckets(m_buckets_data.empty() ? static_empty_bucket_ptr() : m_buckets_data.data()),
                      m_bucket_count(other.m_bucket_count),
                      m_nb_elements(other.m_nb_elements),
                      m_load_threshold(other.m_load_threshold),
                      m_min_load_factor(other.m_min_load_factor),
                      m_max_load_factor(other.m_max_load_factor),
                      m_grow_on_next_insert(other.m_grow_on_next_insert),
                      m_try_skrink_on_next_insert(other.m_try_skrink_on_next_insert) {
                other.clear_and_shrink();
            }

            robin_hash &operator=(const robin_hash &other) {
                if (&other != this) {
                    Hash::operator=(other);
                    KeyEqual::operator=(other);
                    GrowthPolicy::operator=(other);

                    m_buckets_data = other.m_buckets_data;
                    m_buckets = m_buckets_data.empty() ? static_empty_bucket_ptr() :
                                m_buckets_data.data();
                    m_bucket_count = other.m_bucket_count;
                    m_nb_elements = other.m_nb_elements;

                    m_load_threshold = other.m_load_threshold;
                    m_min_load_factor = other.m_min_load_factor;
                    m_max_load_factor = other.m_max_load_factor;

                    m_grow_on_next_insert = other.m_grow_on_next_insert;
                    m_try_skrink_on_next_insert = other.m_try_skrink_on_next_insert;
                }

                return *this;
            }

            robin_hash &operator=(robin_hash &&other) {
                other.swap(*this);
                other.clear();

                return *this;
            }

            allocator_type get_allocator() const {
                return m_buckets_data.get_allocator();
            }


            /*
             * Iterators
             */
            iterator begin() noexcept {
                std::size_t i = 0;
                while (i < m_bucket_count && m_buckets[i].empty()) {
                    i++;
                }

                return iterator(m_buckets + i);
            }

            const_iterator begin() const noexcept {
                return cbegin();
            }

            const_iterator cbegin() const noexcept {
                std::size_t i = 0;
                while (i < m_bucket_count && m_buckets[i].empty()) {
                    i++;
                }

                return const_iterator(m_buckets + i);
            }

            iterator end() noexcept {
                return iterator(m_buckets + m_bucket_count);
            }

            const_iterator end() const noexcept {
                return cend();
            }

            const_iterator cend() const noexcept {
                return const_iterator(m_buckets + m_bucket_count);
            }


            /*
             * Capacity
             */
            bool empty() const noexcept {
                return m_nb_elements == 0;
            }

            size_type size() const noexcept {
                return m_nb_elements;
            }

            size_type max_size() const noexcept {
                return m_buckets_data.max_size();
            }

            /*
             * Modifiers
             */
            void clear() noexcept {
                if (m_min_load_factor > 0.0f) {
                    clear_and_shrink();
                } else {
                    for (auto &bucket: m_buckets_data) {
                        bucket.clear();
                    }

                    m_nb_elements = 0;
                    m_grow_on_next_insert = false;
                }
            }


            template<typename P>
            std::pair<iterator, bool> insert(P &&value) {
                return insert_impl(KeySelect()(value), std::forward<P>(value));
            }

            template<typename P>
            iterator insert_hint(const_iterator hint, P &&value) {
                if (hint != cend() && compare_keys(KeySelect()(*hint), KeySelect()(value))) {
                    return mutable_iterator(hint);
                }

                return insert(std::forward<P>(value)).first;
            }

            template<class InputIt>
            void insert(InputIt first, InputIt last) {
                if (std::is_base_of<std::forward_iterator_tag,
                        typename std::iterator_traits<InputIt>::iterator_category>::value) {
                    const auto nb_elements_insert = std::distance(first, last);
                    const size_type nb_free_buckets = m_load_threshold - size();
                    tsl_rh_assert(m_load_threshold >= size());

                    if (nb_elements_insert > 0 && nb_free_buckets < size_type(nb_elements_insert)) {
                        reserve(size() + size_type(nb_elements_insert));
                    }
                }

                for (; first != last; ++first) {
                    insert(*first);
                }
            }


            template<class K, class M>
            std::pair<iterator, bool> insert_or_assign(K &&key, M &&obj) {
                auto it = try_emplace(std::forward<K>(key), std::forward<M>(obj));
                if (!it.second) {
                    it.first.value() = std::forward<M>(obj);
                }

                return it;
            }

            template<class K, class M>
            iterator insert_or_assign(const_iterator hint, K &&key, M &&obj) {
                if (hint != cend() && compare_keys(KeySelect()(*hint), key)) {
                    auto it = mutable_iterator(hint);
                    it.value() = std::forward<M>(obj);

                    return it;
                }

                return insert_or_assign(std::forward<K>(key), std::forward<M>(obj)).first;
            }


            template<class... Args>
            std::pair<iterator, bool> emplace(Args &&... args) {
                return insert(value_type(std::forward<Args>(args)...));
            }

            template<class... Args>
            iterator emplace_hint(const_iterator hint, Args &&... args) {
                return insert_hint(hint, value_type(std::forward<Args>(args)...));
            }


            template<class K, class... Args>
            std::pair<iterator, bool> try_emplace(K &&key, Args &&... args) {
                return insert_impl(key, std::piecewise_construct,
                                   std::forward_as_tuple(std::forward<K>(key)),
                                   std::forward_as_tuple(std::forward<Args>(args)...));
            }

            template<class K, class... Args>
            iterator try_emplace_hint(const_iterator hint, K &&key, Args &&... args) {
                if (hint != cend() && compare_keys(KeySelect()(*hint), key)) {
                    return mutable_iterator(hint);
                }

                return try_emplace(std::forward<K>(key), std::forward<Args>(args)...).first;
            }

            /**
             * Here to avoid `template<class K> size_type erase(const K& key)` being used when
             * we use an `iterator` instead of a `const_iterator`.
             */
            iterator erase(iterator pos) {
                erase_from_bucket(pos);

                /**
                 * Erase bucket used a backward shift after clearing the bucket.
                 * Check if there is a new value in the bucket, if not get the next non-empty.
                 */
                if (pos.m_bucket->empty()) {
                    ++pos;
                }

                m_try_skrink_on_next_insert = true;

                return pos;
            }

            iterator erase(const_iterator pos) {
                return erase(mutable_iterator(pos));
            }

            iterator erase(const_iterator first, const_iterator last) {
                if (first == last) {
                    return mutable_iterator(first);
                }

                auto first_mutable = mutable_iterator(first);
                auto last_mutable = mutable_iterator(last);
                for (auto it = first_mutable.m_bucket; it != last_mutable.m_bucket; ++it) {
                    if (!it->empty()) {
                        it->clear();
                        m_nb_elements--;
                    }
                }

                if (last_mutable == end()) {
                    m_try_skrink_on_next_insert = true;
                    return end();
                }


                /*
                 * Backward shift on the values which come after the deleted values.
                 * We try to move the values closer to their ideal bucket.
                 */
                std::size_t icloser_bucket = static_cast<std::size_t>(first_mutable.m_bucket - m_buckets);
                std::size_t ito_move_closer_value = static_cast<std::size_t>(last_mutable.m_bucket - m_buckets);
                tsl_rh_assert(ito_move_closer_value > icloser_bucket);

                const std::size_t ireturn_bucket = ito_move_closer_value -
                                                   std::min(ito_move_closer_value - icloser_bucket,
                                                            std::size_t(
                                                                    m_buckets[ito_move_closer_value].dist_from_ideal_bucket()));

                while (ito_move_closer_value < m_bucket_count &&
                       m_buckets[ito_move_closer_value].dist_from_ideal_bucket() > 0) {
                    icloser_bucket = ito_move_closer_value -
                                     std::min(ito_move_closer_value - icloser_bucket,
                                              std::size_t(m_buckets[ito_move_closer_value].dist_from_ideal_bucket()));


                    tsl_rh_assert(m_buckets[icloser_bucket].empty());
                    const distance_type new_distance = distance_type(
                            m_buckets[ito_move_closer_value].dist_from_ideal_bucket() -
                            (ito_move_closer_value - icloser_bucket));
                    m_buckets[icloser_bucket].set_value_of_empty_bucket(new_distance,
                                                                        m_buckets[ito_move_closer_value].truncated_hash(),
                                                                        std::move(
                                                                                m_buckets[ito_move_closer_value].value()));
                    m_buckets[ito_move_closer_value].clear();


                    ++icloser_bucket;
                    ++ito_move_closer_value;
                }

                m_try_skrink_on_next_insert = true;

                return iterator(m_buckets + ireturn_bucket);
            }


            template<class K>
            size_type erase(const K &key) {
                return erase(key, hash_key(key));
            }

            template<class K>
            size_type erase(const K &key, std::size_t hash) {
                auto it = find(key, hash);
                if (it != end()) {
                    erase_from_bucket(it);
                    m_try_skrink_on_next_insert = true;

                    return 1;
                } else {
                    return 0;
                }
            }


            void swap(robin_hash &other) {
                using std::swap;

                swap(static_cast<Hash &>(*this), static_cast<Hash &>(other));
                swap(static_cast<KeyEqual &>(*this), static_cast<KeyEqual &>(other));
                swap(static_cast<GrowthPolicy &>(*this), static_cast<GrowthPolicy &>(other));
                swap(m_buckets_data, other.m_buckets_data);
                swap(m_buckets, other.m_buckets);
                swap(m_bucket_count, other.m_bucket_count);
                swap(m_nb_elements, other.m_nb_elements);
                swap(m_load_threshold, other.m_load_threshold);
                swap(m_min_load_factor, other.m_min_load_factor);
                swap(m_max_load_factor, other.m_max_load_factor);
                swap(m_grow_on_next_insert, other.m_grow_on_next_insert);
                swap(m_try_skrink_on_next_insert, other.m_try_skrink_on_next_insert);
            }


            /*
             * Lookup
             */
            template<class K, class U = ValueSelect, typename std::enable_if<has_mapped_type<U>::value>::type * = nullptr>
            typename U::value_type &at(const K &key) {
                return at(key, hash_key(key));
            }

            template<class K, class U = ValueSelect, typename std::enable_if<has_mapped_type<U>::value>::type * = nullptr>
            typename U::value_type &at(const K &key, std::size_t hash) {
                return const_cast<typename U::value_type &>(static_cast<const robin_hash *>(this)->at(key, hash));
            }


            template<class K, class U = ValueSelect, typename std::enable_if<has_mapped_type<U>::value>::type * = nullptr>
            const typename U::value_type &at(const K &key) const {
                return at(key, hash_key(key));
            }

            template<class K, class U = ValueSelect, typename std::enable_if<has_mapped_type<U>::value>::type * = nullptr>
            const typename U::value_type &at(const K &key, std::size_t hash) const {
                auto it = find(key, hash);
                if (it != cend()) {
                    return it.value();
                } else {
                    TSL_RH_THROW_OR_TERMINATE(std::out_of_range, "Couldn't find key.");
                }
            }

            template<class K, class U = ValueSelect, typename std::enable_if<has_mapped_type<U>::value>::type * = nullptr>
            typename U::value_type &operator[](K &&key) {
                return try_emplace(std::forward<K>(key)).first.value();
            }


            template<class K>
            size_type count(const K &key) const {
                return count(key, hash_key(key));
            }

            template<class K>
            size_type count(const K &key, std::size_t hash) const {
                if (find(key, hash) != cend()) {
                    return 1;
                } else {
                    return 0;
                }
            }


            template<class K>
            iterator find(const K &key) {
                return find_impl(key, hash_key(key));
            }

            template<class K>
            iterator find(const K &key, std::size_t hash) {
                return find_impl(key, hash);
            }


            template<class K>
            const_iterator find(const K &key) const {
                return find_impl(key, hash_key(key));
            }

            template<class K>
            const_iterator find(const K &key, std::size_t hash) const {
                return find_impl(key, hash);
            }


            template<class K>
            bool contains(const K &key) const {
                return contains(key, hash_key(key));
            }

            template<class K>
            bool contains(const K &key, std::size_t hash) const {
                return count(key, hash) != 0;
            }


            template<class K>
            std::pair<iterator, iterator> equal_range(const K &key) {
                return equal_range(key, hash_key(key));
            }

            template<class K>
            std::pair<iterator, iterator> equal_range(const K &key, std::size_t hash) {
                iterator it = find(key, hash);
                return std::make_pair(it, (it == end()) ? it : std::next(it));
            }


            template<class K>
            std::pair<const_iterator, const_iterator> equal_range(const K &key) const {
                return equal_range(key, hash_key(key));
            }

            template<class K>
            std::pair<const_iterator, const_iterator> equal_range(const K &key, std::size_t hash) const {
                const_iterator it = find(key, hash);
                return std::make_pair(it, (it == cend()) ? it : std::next(it));
            }

            /*
             * Bucket interface
             */
            size_type bucket_count() const {
                return m_bucket_count;
            }

            size_type max_bucket_count() const {
                return std::min(GrowthPolicy::max_bucket_count(), m_buckets_data.max_size());
            }

            /*
             * Hash policy
             */
            float load_factor() const {
                if (bucket_count() == 0) {
                    return 0;
                }

                return float(m_nb_elements) / float(bucket_count());
            }

            float min_load_factor() const {
                return m_min_load_factor;
            }

            float max_load_factor() const {
                return m_max_load_factor;
            }

            void min_load_factor(float ml) {
                m_min_load_factor = clamp(ml, float(MINIMUM_MIN_LOAD_FACTOR),
                                          float(MAXIMUM_MIN_LOAD_FACTOR));
            }

            void max_load_factor(float ml) {
                m_max_load_factor = clamp(ml, float(MINIMUM_MAX_LOAD_FACTOR),
                                          float(MAXIMUM_MAX_LOAD_FACTOR));
                m_load_threshold = size_type(float(bucket_count()) * m_max_load_factor);
            }

            void rehash(size_type count) {
                count = std::max(count, size_type(std::ceil(float(size()) / max_load_factor())));
                rehash_impl(count);
            }

            void reserve(size_type count) {
                rehash(size_type(std::ceil(float(count) / max_load_factor())));
            }

            /*
             * Observers
             */
            hasher hash_function() const {
                return static_cast<const Hash &>(*this);
            }

            key_equal key_eq() const {
                return static_cast<const KeyEqual &>(*this);
            }


            /*
             * Other
             */
            iterator mutable_iterator(const_iterator pos) {
                return iterator(const_cast<bucket_entry *>(pos.m_bucket));
            }

        private:
            template<class K>
            std::size_t hash_key(const K &key) const {
                return Hash::operator()(key);
            }

            template<class K1, class K2>
            bool compare_keys(const K1 &key1, const K2 &key2) const {
                return KeyEqual::operator()(key1, key2);
            }

            std::size_t bucket_for_hash(std::size_t hash) const {
                const std::size_t bucket = GrowthPolicy::bucket_for_hash(hash);
                tsl_rh_assert(bucket < m_bucket_count || (bucket == 0 && m_bucket_count == 0));

                return bucket;
            }

            template<class U = GrowthPolicy, typename std::enable_if<is_power_of_two_policy<U>::value>::type * = nullptr>
            std::size_t next_bucket(std::size_t index) const noexcept {
                tsl_rh_assert(index < bucket_count());

                return (index + 1) & this->m_mask;
            }

            template<class U = GrowthPolicy, typename std::enable_if<!is_power_of_two_policy<U>::value>::type * = nullptr>
            std::size_t next_bucket(std::size_t index) const noexcept {
                tsl_rh_assert(index < bucket_count());

                index++;
                return (index != bucket_count()) ? index : 0;
            }


            template<class K>
            iterator find_impl(const K &key, std::size_t hash) {
                return mutable_iterator(static_cast<const robin_hash *>(this)->find(key, hash));
            }

            template<class K>
            const_iterator find_impl(const K &key, std::size_t hash) const {
                std::size_t ibucket = bucket_for_hash(hash);
                distance_type dist_from_ideal_bucket = 0;

                while (dist_from_ideal_bucket <= m_buckets[ibucket].dist_from_ideal_bucket()) {
                    if (TSL_RH_LIKELY((!USE_STORED_HASH_ON_LOOKUP || m_buckets[ibucket].bucket_hash_equal(hash)) &&
                                      compare_keys(KeySelect()(m_buckets[ibucket].value()), key))) {
                        return const_iterator(m_buckets + ibucket);
                    }

                    ibucket = next_bucket(ibucket);
                    dist_from_ideal_bucket++;
                }

                return cend();
            }

            void erase_from_bucket(iterator pos) {
                pos.m_bucket->clear();
                m_nb_elements--;

                /**
                 * Backward shift, swap the empty bucket, previous_ibucket, with the values on its right, ibucket,
                 * until we cross another empty bucket or if the other bucket has a distance_from_ideal_bucket == 0.
                 *
                 * We try to move the values closer to their ideal bucket.
                 */
                std::size_t previous_ibucket = static_cast<std::size_t>(pos.m_bucket - m_buckets);
                std::size_t ibucket = next_bucket(previous_ibucket);

                while (m_buckets[ibucket].dist_from_ideal_bucket() > 0) {
                    tsl_rh_assert(m_buckets[previous_ibucket].empty());

                    const distance_type new_distance = distance_type(m_buckets[ibucket].dist_from_ideal_bucket() - 1);
                    m_buckets[previous_ibucket].set_value_of_empty_bucket(new_distance,
                                                                          m_buckets[ibucket].truncated_hash(),
                                                                          std::move(m_buckets[ibucket].value()));
                    m_buckets[ibucket].clear();

                    previous_ibucket = ibucket;
                    ibucket = next_bucket(ibucket);
                }
            }

            template<class K, class... Args>
            std::pair<iterator, bool> insert_impl(const K &key, Args &&... value_type_args) {
                const std::size_t hash = hash_key(key);

                std::size_t ibucket = bucket_for_hash(hash);
                distance_type dist_from_ideal_bucket = 0;

                while (dist_from_ideal_bucket <= m_buckets[ibucket].dist_from_ideal_bucket()) {
                    if ((!USE_STORED_HASH_ON_LOOKUP || m_buckets[ibucket].bucket_hash_equal(hash)) &&
                        compare_keys(KeySelect()(m_buckets[ibucket].value()), key)) {
                        return std::make_pair(iterator(m_buckets + ibucket), false);
                    }

                    ibucket = next_bucket(ibucket);
                    dist_from_ideal_bucket++;
                }

                if (rehash_on_extreme_load()) {
                    ibucket = bucket_for_hash(hash);
                    dist_from_ideal_bucket = 0;

                    while (dist_from_ideal_bucket <= m_buckets[ibucket].dist_from_ideal_bucket()) {
                        ibucket = next_bucket(ibucket);
                        dist_from_ideal_bucket++;
                    }
                }


                if (m_buckets[ibucket].empty()) {
                    m_buckets[ibucket].set_value_of_empty_bucket(dist_from_ideal_bucket,
                                                                 bucket_entry::truncate_hash(hash),
                                                                 std::forward<Args>(value_type_args)...);
                } else {
                    insert_value(ibucket, dist_from_ideal_bucket, bucket_entry::truncate_hash(hash),
                                 std::forward<Args>(value_type_args)...);
                }


                m_nb_elements++;
                /*
                 * The value will be inserted in ibucket in any case, either because it was
                 * empty or by stealing the bucket (robin hood).
                 */
                return std::make_pair(iterator(m_buckets + ibucket), true);
            }


            template<class... Args>
            void insert_value(std::size_t ibucket, distance_type dist_from_ideal_bucket,
                              truncated_hash_type hash, Args &&... value_type_args) {
                value_type value(std::forward<Args>(value_type_args)...);
                insert_value_impl(ibucket, dist_from_ideal_bucket, hash, value);
            }

            void insert_value(std::size_t ibucket, distance_type dist_from_ideal_bucket,
                              truncated_hash_type hash, value_type &&value) {
                insert_value_impl(ibucket, dist_from_ideal_bucket, hash, value);
            }

            /*
             * We don't use `value_type&& value` as last argument due to a bug in MSVC when `value_type` is a pointer,
             * The compiler is not able to see the difference between `std::string*` and `std::string*&&` resulting in
             * a compilation error.
             *
             * The `value` will be in a moved state at the end of the function.
             */
            void insert_value_impl(std::size_t ibucket, distance_type dist_from_ideal_bucket,
                                   truncated_hash_type hash, value_type &value) {
                m_buckets[ibucket].swap_with_value_in_bucket(dist_from_ideal_bucket, hash, value);
                ibucket = next_bucket(ibucket);
                dist_from_ideal_bucket++;

                while (!m_buckets[ibucket].empty()) {
                    if (dist_from_ideal_bucket > m_buckets[ibucket].dist_from_ideal_bucket()) {
                        if (dist_from_ideal_bucket >= bucket_entry::DIST_FROM_IDEAL_BUCKET_LIMIT) {
                            /**
                             * The number of probes is really high, rehash the map on the next insert.
                             * Difficult to do now as rehash may throw an exception.
                             */
                            m_grow_on_next_insert = true;
                        }

                        m_buckets[ibucket].swap_with_value_in_bucket(dist_from_ideal_bucket, hash, value);
                    }

                    ibucket = next_bucket(ibucket);
                    dist_from_ideal_bucket++;
                }

                m_buckets[ibucket].set_value_of_empty_bucket(dist_from_ideal_bucket, hash, std::move(value));
            }


            void rehash_impl(size_type count) {
                robin_hash new_table(count, static_cast<Hash &>(*this), static_cast<KeyEqual &>(*this),
                                     get_allocator(), m_min_load_factor, m_max_load_factor);

                const bool use_stored_hash = USE_STORED_HASH_ON_REHASH(new_table.bucket_count());
                for (auto &bucket: m_buckets_data) {
                    if (bucket.empty()) {
                        continue;
                    }

                    const std::size_t hash = use_stored_hash ? bucket.truncated_hash() :
                                             new_table.hash_key(KeySelect()(bucket.value()));

                    new_table.insert_value_on_rehash(new_table.bucket_for_hash(hash), 0,
                                                     bucket_entry::truncate_hash(hash), std::move(bucket.value()));
                }

                new_table.m_nb_elements = m_nb_elements;
                new_table.swap(*this);
            }

            void clear_and_shrink() noexcept {
                GrowthPolicy::clear();
                m_buckets_data.clear();
                m_buckets = static_empty_bucket_ptr();
                m_bucket_count = 0;
                m_nb_elements = 0;
                m_load_threshold = 0;
                m_grow_on_next_insert = false;
                m_try_skrink_on_next_insert = false;
            }

            void insert_value_on_rehash(std::size_t ibucket, distance_type dist_from_ideal_bucket,
                                        truncated_hash_type hash, value_type &&value) {
                while (true) {
                    if (dist_from_ideal_bucket > m_buckets[ibucket].dist_from_ideal_bucket()) {
                        if (m_buckets[ibucket].empty()) {
                            m_buckets[ibucket].set_value_of_empty_bucket(dist_from_ideal_bucket, hash,
                                                                         std::move(value));
                            return;
                        } else {
                            m_buckets[ibucket].swap_with_value_in_bucket(dist_from_ideal_bucket, hash, value);
                        }
                    }

                    dist_from_ideal_bucket++;
                    ibucket = next_bucket(ibucket);
                }
            }


            /**
             * Grow the table if m_grow_on_next_insert is true or we reached the max_load_factor.
             * Shrink the table if m_try_skrink_on_next_insert is true (an erase occured) and
             * we're below the min_load_factor.
             *
             * Return true if the table has been rehashed.
             */
            bool rehash_on_extreme_load() {
                if (m_grow_on_next_insert || size() >= m_load_threshold) {
                    rehash_impl(GrowthPolicy::next_bucket_count());
                    m_grow_on_next_insert = false;

                    return true;
                }

                if (m_try_skrink_on_next_insert) {
                    m_try_skrink_on_next_insert = false;
                    if (m_min_load_factor != 0.0f && load_factor() < m_min_load_factor) {
                        reserve(size() + 1);

                        return true;
                    }
                }

                return false;
            }


        public:
            static const size_type DEFAULT_INIT_BUCKETS_SIZE = 0;

            static constexpr float DEFAULT_MAX_LOAD_FACTOR = 0.5f;
            static constexpr float MINIMUM_MAX_LOAD_FACTOR = 0.2f;
            static constexpr float MAXIMUM_MAX_LOAD_FACTOR = 0.95f;

            static constexpr float DEFAULT_MIN_LOAD_FACTOR = 0.0f;
            static constexpr float MINIMUM_MIN_LOAD_FACTOR = 0.0f;
            static constexpr float MAXIMUM_MIN_LOAD_FACTOR = 0.15f;

            static_assert(MINIMUM_MAX_LOAD_FACTOR < MAXIMUM_MAX_LOAD_FACTOR,
                          "MINIMUM_MAX_LOAD_FACTOR should be < MAXIMUM_MAX_LOAD_FACTOR");
            static_assert(MINIMUM_MIN_LOAD_FACTOR < MAXIMUM_MIN_LOAD_FACTOR,
                          "MINIMUM_MIN_LOAD_FACTOR should be < MAXIMUM_MIN_LOAD_FACTOR");
            static_assert(MAXIMUM_MIN_LOAD_FACTOR < MINIMUM_MAX_LOAD_FACTOR,
                          "MAXIMUM_MIN_LOAD_FACTOR should be < MINIMUM_MAX_LOAD_FACTOR");

        private:
            /**
             * Return an always valid pointer to an static empty bucket_entry with last_bucket() == true.
             */
            bucket_entry *static_empty_bucket_ptr() noexcept {
                static bucket_entry empty_bucket(true);
                return &empty_bucket;
            }

        private:
            buckets_container_type m_buckets_data;

            /**
             * Points to m_buckets_data.data() if !m_buckets_data.empty() otherwise points to static_empty_bucket_ptr.
             * This variable is useful to avoid the cost of checking if m_buckets_data is empty when trying
             * to find an element.
             *
             * TODO Remove m_buckets_data and only use a pointer instead of a pointer+vector to save some space in the robin_hash object.
             * Manage the Allocator manually.
             */
            bucket_entry *m_buckets;

            /**
             * Used a lot in find, avoid the call to m_buckets_data.size() which is a bit slower.
             */
            size_type m_bucket_count;

            size_type m_nb_elements;

            size_type m_load_threshold;

            float m_min_load_factor;
            float m_max_load_factor;

            bool m_grow_on_next_insert;

            /**
             * We can't shrink down the map on erase operations as the erase methods need to return the next iterator.
             * Shrinking the map would invalidate all the iterators and we could not return the next iterator in a meaningful way,
             * On erase, we thus just indicate on erase that we should try to shrink the hash table on the next insert
             * if we go below the min_load_factor.
             */
            bool m_try_skrink_on_next_insert;
        };

    }

}
#endif
namespace tsl {
    template<class Key,
            class T,
            class Hash = std::hash<Key>,
            class KeyEqual = std::equal_to<Key>,
            class Allocator = std::allocator<std::pair<Key, T>>,
            bool StoreHash = false,
            class GrowthPolicy = tsl::rh::power_of_two_growth_policy<2>>
    class robin_map {
    private:
        template<typename U>
        using has_is_transparent = tsl::detail_robin_hash::has_is_transparent<U>;

        class KeySelect {
        public:
            using key_type = Key;

            const key_type &operator()(const std::pair<Key, T> &key_value) const noexcept {
                return key_value.first;
            }

            key_type &operator()(std::pair<Key, T> &key_value) noexcept {
                return key_value.first;
            }
        };

        class ValueSelect {
        public:
            using value_type = T;

            const value_type &operator()(const std::pair<Key, T> &key_value) const noexcept {
                return key_value.second;
            }

            value_type &operator()(std::pair<Key, T> &key_value) noexcept {
                return key_value.second;
            }
        };

        using ht = detail_robin_hash::robin_hash<std::pair<Key, T>, KeySelect, ValueSelect,
                Hash, KeyEqual, Allocator, StoreHash, GrowthPolicy>;

    public:
        using key_type = typename ht::key_type;
        using mapped_type = T;
        using value_type = typename ht::value_type;
        using size_type = typename ht::size_type;
        using difference_type = typename ht::difference_type;
        using hasher = typename ht::hasher;
        using key_equal = typename ht::key_equal;
        using allocator_type = typename ht::allocator_type;
        using reference = typename ht::reference;
        using const_reference = typename ht::const_reference;
        using pointer = typename ht::pointer;
        using const_pointer = typename ht::const_pointer;
        using iterator = typename ht::iterator;
        using const_iterator = typename ht::const_iterator;


    public:
        /*
         * Constructors
         */
        robin_map() : robin_map(ht::DEFAULT_INIT_BUCKETS_SIZE) {
        }

        explicit robin_map(size_type bucket_count,
                           const Hash &hash = Hash(),
                           const KeyEqual &equal = KeyEqual(),
                           const Allocator &alloc = Allocator()) :
                m_ht(bucket_count, hash, equal, alloc) {
        }

        robin_map(size_type bucket_count,
                  const Allocator &alloc) : robin_map(bucket_count, Hash(), KeyEqual(), alloc) {
        }

        robin_map(size_type bucket_count,
                  const Hash &hash,
                  const Allocator &alloc) : robin_map(bucket_count, hash, KeyEqual(), alloc) {
        }

        explicit robin_map(const Allocator &alloc) : robin_map(ht::DEFAULT_INIT_BUCKETS_SIZE, alloc) {
        }

        template<class InputIt>
        robin_map(InputIt first, InputIt last,
                  size_type bucket_count = ht::DEFAULT_INIT_BUCKETS_SIZE,
                  const Hash &hash = Hash(),
                  const KeyEqual &equal = KeyEqual(),
                  const Allocator &alloc = Allocator()): robin_map(bucket_count, hash, equal, alloc) {
            insert(first, last);
        }

        template<class InputIt>
        robin_map(InputIt first, InputIt last,
                  size_type bucket_count,
                  const Allocator &alloc): robin_map(first, last, bucket_count, Hash(), KeyEqual(), alloc) {
        }

        template<class InputIt>
        robin_map(InputIt first, InputIt last,
                  size_type bucket_count,
                  const Hash &hash,
                  const Allocator &alloc): robin_map(first, last, bucket_count, hash, KeyEqual(), alloc) {
        }

        robin_map(std::initializer_list<value_type> init,
                  size_type bucket_count = ht::DEFAULT_INIT_BUCKETS_SIZE,
                  const Hash &hash = Hash(),
                  const KeyEqual &equal = KeyEqual(),
                  const Allocator &alloc = Allocator()) :
                robin_map(init.begin(), init.end(), bucket_count, hash, equal, alloc) {
        }

        robin_map(std::initializer_list<value_type> init,
                  size_type bucket_count,
                  const Allocator &alloc) :
                robin_map(init.begin(), init.end(), bucket_count, Hash(), KeyEqual(), alloc) {
        }

        robin_map(std::initializer_list<value_type> init,
                  size_type bucket_count,
                  const Hash &hash,
                  const Allocator &alloc) :
                robin_map(init.begin(), init.end(), bucket_count, hash, KeyEqual(), alloc) {
        }

        robin_map &operator=(std::initializer_list<value_type> ilist) {
            m_ht.clear();

            m_ht.reserve(ilist.size());
            m_ht.insert(ilist.begin(), ilist.end());

            return *this;
        }

        allocator_type get_allocator() const { return m_ht.get_allocator(); }


        /*
         * Iterators
         */
        iterator begin() noexcept { return m_ht.begin(); }

        const_iterator begin() const noexcept { return m_ht.begin(); }

        const_iterator cbegin() const noexcept { return m_ht.cbegin(); }

        iterator end() noexcept { return m_ht.end(); }

        const_iterator end() const noexcept { return m_ht.end(); }

        const_iterator cend() const noexcept { return m_ht.cend(); }


        /*
         * Capacity
         */
        bool empty() const noexcept { return m_ht.empty(); }

        size_type size() const noexcept { return m_ht.size(); }

        size_type max_size() const noexcept { return m_ht.max_size(); }

        /*
         * Modifiers
         */
        void clear() noexcept { m_ht.clear(); }


        std::pair<iterator, bool> insert(const value_type &value) {
            return m_ht.insert(value);
        }

        template<class P, typename std::enable_if<std::is_constructible<value_type, P &&>::value>::type * = nullptr>
        std::pair<iterator, bool> insert(P &&value) {
            return m_ht.emplace(std::forward<P>(value));
        }

        std::pair<iterator, bool> insert(value_type &&value) {
            return m_ht.insert(std::move(value));
        }


        iterator insert(const_iterator hint, const value_type &value) {
            return m_ht.insert_hint(hint, value);
        }

        template<class P, typename std::enable_if<std::is_constructible<value_type, P &&>::value>::type * = nullptr>
        iterator insert(const_iterator hint, P &&value) {
            return m_ht.emplace_hint(hint, std::forward<P>(value));
        }

        iterator insert(const_iterator hint, value_type &&value) {
            return m_ht.insert_hint(hint, std::move(value));
        }


        template<class InputIt>
        void insert(InputIt first, InputIt last) {
            m_ht.insert(first, last);
        }

        void insert(std::initializer_list<value_type> ilist) {
            m_ht.insert(ilist.begin(), ilist.end());
        }


        template<class M>
        std::pair<iterator, bool> insert_or_assign(const key_type &k, M &&obj) {
            return m_ht.insert_or_assign(k, std::forward<M>(obj));
        }

        template<class M>
        std::pair<iterator, bool> insert_or_assign(key_type &&k, M &&obj) {
            return m_ht.insert_or_assign(std::move(k), std::forward<M>(obj));
        }

        template<class M>
        iterator insert_or_assign(const_iterator hint, const key_type &k, M &&obj) {
            return m_ht.insert_or_assign(hint, k, std::forward<M>(obj));
        }

        template<class M>
        iterator insert_or_assign(const_iterator hint, key_type &&k, M &&obj) {
            return m_ht.insert_or_assign(hint, std::move(k), std::forward<M>(obj));
        }


        /**
         * Due to the way elements are stored, emplace will need to move or copy the key-value once.
         * The method is equivalent to insert(value_type(std::forward<Args>(args)...));
         *
         * Mainly here for compatibility with the std::unordered_map interface.
         */
        template<class... Args>
        std::pair<iterator, bool> emplace(Args &&... args) {
            return m_ht.emplace(std::forward<Args>(args)...);
        }


        /**
         * Due to the way elements are stored, emplace_hint will need to move or copy the key-value once.
         * The method is equivalent to insert(hint, value_type(std::forward<Args>(args)...));
         *
         * Mainly here for compatibility with the std::unordered_map interface.
         */
        template<class... Args>
        iterator emplace_hint(const_iterator hint, Args &&... args) {
            return m_ht.emplace_hint(hint, std::forward<Args>(args)...);
        }


        template<class... Args>
        std::pair<iterator, bool> try_emplace(const key_type &k, Args &&... args) {
            return m_ht.try_emplace(k, std::forward<Args>(args)...);
        }

        template<class... Args>
        std::pair<iterator, bool> try_emplace(key_type &&k, Args &&... args) {
            return m_ht.try_emplace(std::move(k), std::forward<Args>(args)...);
        }

        template<class... Args>
        iterator try_emplace(const_iterator hint, const key_type &k, Args &&... args) {
            return m_ht.try_emplace_hint(hint, k, std::forward<Args>(args)...);
        }

        template<class... Args>
        iterator try_emplace(const_iterator hint, key_type &&k, Args &&... args) {
            return m_ht.try_emplace_hint(hint, std::move(k), std::forward<Args>(args)...);
        }


        iterator erase(iterator pos) { return m_ht.erase(pos); }

        iterator erase(const_iterator pos) { return m_ht.erase(pos); }

        iterator erase(const_iterator first, const_iterator last) { return m_ht.erase(first, last); }

        size_type erase(const key_type &key) { return m_ht.erase(key); }

        /**
         * Use the hash value 'precalculated_hash' instead of hashing the key. The hash value should be the same
         * as hash_function()(key). Usefull to speed-up the lookup to the value if you already have the hash.
         */
        size_type erase(const key_type &key, std::size_t precalculated_hash) {
            return m_ht.erase(key, precalculated_hash);
        }

        /**
         * This overload only participates in the overload resolution if the typedef KeyEqual::is_transparent exists.
         * If so, K must be hashable and comparable to Key.
         */
        template<class K, class KE = KeyEqual, typename std::enable_if<has_is_transparent<KE>::value>::type * = nullptr>
        size_type erase(const K &key) { return m_ht.erase(key); }

        /**
         * @copydoc erase(const K& key)
         *
         * Use the hash value 'precalculated_hash' instead of hashing the key. The hash value should be the same
         * as hash_function()(key). Usefull to speed-up the lookup to the value if you already have the hash.
         */
        template<class K, class KE = KeyEqual, typename std::enable_if<has_is_transparent<KE>::value>::type * = nullptr>
        size_type erase(const K &key, std::size_t precalculated_hash) {
            return m_ht.erase(key, precalculated_hash);
        }


        void swap(robin_map &other) { other.m_ht.swap(m_ht); }


        /*
         * Lookup
         */
        T &at(const Key &key) { return m_ht.at(key); }

        /**
         * Use the hash value 'precalculated_hash' instead of hashing the key. The hash value should be the same
         * as hash_function()(key). Usefull to speed-up the lookup if you already have the hash.
         */
        T &at(const Key &key, std::size_t precalculated_hash) { return m_ht.at(key, precalculated_hash); }


        const T &at(const Key &key) const { return m_ht.at(key); }

        /**
         * @copydoc at(const Key& key, std::size_t precalculated_hash)
         */
        const T &at(const Key &key, std::size_t precalculated_hash) const { return m_ht.at(key, precalculated_hash); }


        /**
         * This overload only participates in the overload resolution if the typedef KeyEqual::is_transparent exists.
         * If so, K must be hashable and comparable to Key.
         */
        template<class K, class KE = KeyEqual, typename std::enable_if<has_is_transparent<KE>::value>::type * = nullptr>
        T &at(const K &key) { return m_ht.at(key); }

        /**
         * @copydoc at(const K& key)
         *
         * Use the hash value 'precalculated_hash' instead of hashing the key. The hash value should be the same
         * as hash_function()(key). Usefull to speed-up the lookup if you already have the hash.
         */
        template<class K, class KE = KeyEqual, typename std::enable_if<has_is_transparent<KE>::value>::type * = nullptr>
        T &at(const K &key, std::size_t precalculated_hash) { return m_ht.at(key, precalculated_hash); }


        /**
         * @copydoc at(const K& key)
         */
        template<class K, class KE = KeyEqual, typename std::enable_if<has_is_transparent<KE>::value>::type * = nullptr>
        const T &at(const K &key) const { return m_ht.at(key); }

        /**
         * @copydoc at(const K& key, std::size_t precalculated_hash)
         */
        template<class K, class KE = KeyEqual, typename std::enable_if<has_is_transparent<KE>::value>::type * = nullptr>
        const T &at(const K &key, std::size_t precalculated_hash) const { return m_ht.at(key, precalculated_hash); }


        T &operator[](const Key &key) { return m_ht[key]; }

        T &operator[](Key &&key) { return m_ht[std::move(key)]; }


        size_type count(const Key &key) const { return m_ht.count(key); }

        /**
         * Use the hash value 'precalculated_hash' instead of hashing the key. The hash value should be the same
         * as hash_function()(key). Usefull to speed-up the lookup if you already have the hash.
         */
        size_type count(const Key &key, std::size_t precalculated_hash) const {
            return m_ht.count(key, precalculated_hash);
        }

        /**
         * This overload only participates in the overload resolution if the typedef KeyEqual::is_transparent exists.
         * If so, K must be hashable and comparable to Key.
         */
        template<class K, class KE = KeyEqual, typename std::enable_if<has_is_transparent<KE>::value>::type * = nullptr>
        size_type count(const K &key) const { return m_ht.count(key); }

        /**
         * @copydoc count(const K& key) const
         *
         * Use the hash value 'precalculated_hash' instead of hashing the key. The hash value should be the same
         * as hash_function()(key). Usefull to speed-up the lookup if you already have the hash.
         */
        template<class K, class KE = KeyEqual, typename std::enable_if<has_is_transparent<KE>::value>::type * = nullptr>
        size_type count(const K &key, std::size_t precalculated_hash) const {
            return m_ht.count(key, precalculated_hash);
        }


        iterator find(const Key &key) { return m_ht.find(key); }

        /**
         * Use the hash value 'precalculated_hash' instead of hashing the key. The hash value should be the same
         * as hash_function()(key). Usefull to speed-up the lookup if you already have the hash.
         */
        iterator find(const Key &key, std::size_t precalculated_hash) { return m_ht.find(key, precalculated_hash); }

        const_iterator find(const Key &key) const { return m_ht.find(key); }

        /**
         * @copydoc find(const Key& key, std::size_t precalculated_hash)
         */
        const_iterator find(const Key &key, std::size_t precalculated_hash) const {
            return m_ht.find(key, precalculated_hash);
        }

        /**
         * This overload only participates in the overload resolution if the typedef KeyEqual::is_transparent exists.
         * If so, K must be hashable and comparable to Key.
         */
        template<class K, class KE = KeyEqual, typename std::enable_if<has_is_transparent<KE>::value>::type * = nullptr>
        iterator find(const K &key) { return m_ht.find(key); }

        /**
         * @copydoc find(const K& key)
         *
         * Use the hash value 'precalculated_hash' instead of hashing the key. The hash value should be the same
         * as hash_function()(key). Usefull to speed-up the lookup if you already have the hash.
         */
        template<class K, class KE = KeyEqual, typename std::enable_if<has_is_transparent<KE>::value>::type * = nullptr>
        iterator find(const K &key, std::size_t precalculated_hash) { return m_ht.find(key, precalculated_hash); }

        /**
         * @copydoc find(const K& key)
         */
        template<class K, class KE = KeyEqual, typename std::enable_if<has_is_transparent<KE>::value>::type * = nullptr>
        const_iterator find(const K &key) const { return m_ht.find(key); }

        /**
         * @copydoc find(const K& key)
         *
         * Use the hash value 'precalculated_hash' instead of hashing the key. The hash value should be the same
         * as hash_function()(key). Usefull to speed-up the lookup if you already have the hash.
         */
        template<class K, class KE = KeyEqual, typename std::enable_if<has_is_transparent<KE>::value>::type * = nullptr>
        const_iterator find(const K &key, std::size_t precalculated_hash) const {
            return m_ht.find(key, precalculated_hash);
        }


        bool contains(const Key &key) const { return m_ht.contains(key); }

        /**
         * Use the hash value 'precalculated_hash' instead of hashing the key. The hash value should be the same
         * as hash_function()(key). Usefull to speed-up the lookup if you already have the hash.
         */
        bool contains(const Key &key, std::size_t precalculated_hash) const {
            return m_ht.contains(key, precalculated_hash);
        }

        /**
         * This overload only participates in the overload resolution if the typedef KeyEqual::is_transparent exists.
         * If so, K must be hashable and comparable to Key.
         */
        template<class K, class KE = KeyEqual, typename std::enable_if<has_is_transparent<KE>::value>::type * = nullptr>
        bool contains(const K &key) const { return m_ht.contains(key); }

        /**
         * @copydoc contains(const K& key) const
         *
         * Use the hash value 'precalculated_hash' instead of hashing the key. The hash value should be the same
         * as hash_function()(key). Usefull to speed-up the lookup if you already have the hash.
         */
        template<class K, class KE = KeyEqual, typename std::enable_if<has_is_transparent<KE>::value>::type * = nullptr>
        bool contains(const K &key, std::size_t precalculated_hash) const {
            return m_ht.contains(key, precalculated_hash);
        }


        std::pair<iterator, iterator> equal_range(const Key &key) { return m_ht.equal_range(key); }

        /**
         * Use the hash value 'precalculated_hash' instead of hashing the key. The hash value should be the same
         * as hash_function()(key). Usefull to speed-up the lookup if you already have the hash.
         */
        std::pair<iterator, iterator> equal_range(const Key &key, std::size_t precalculated_hash) {
            return m_ht.equal_range(key, precalculated_hash);
        }

        std::pair<const_iterator, const_iterator> equal_range(const Key &key) const { return m_ht.equal_range(key); }

        /**
         * @copydoc equal_range(const Key& key, std::size_t precalculated_hash)
         */
        std::pair<const_iterator, const_iterator> equal_range(const Key &key, std::size_t precalculated_hash) const {
            return m_ht.equal_range(key, precalculated_hash);
        }

        /**
         * This overload only participates in the overload resolution if the typedef KeyEqual::is_transparent exists.
         * If so, K must be hashable and comparable to Key.
         */
        template<class K, class KE = KeyEqual, typename std::enable_if<has_is_transparent<KE>::value>::type * = nullptr>
        std::pair<iterator, iterator> equal_range(const K &key) { return m_ht.equal_range(key); }


        /**
         * @copydoc equal_range(const K& key)
         *
         * Use the hash value 'precalculated_hash' instead of hashing the key. The hash value should be the same
         * as hash_function()(key). Usefull to speed-up the lookup if you already have the hash.
         */
        template<class K, class KE = KeyEqual, typename std::enable_if<has_is_transparent<KE>::value>::type * = nullptr>
        std::pair<iterator, iterator> equal_range(const K &key, std::size_t precalculated_hash) {
            return m_ht.equal_range(key, precalculated_hash);
        }

        /**
         * @copydoc equal_range(const K& key)
         */
        template<class K, class KE = KeyEqual, typename std::enable_if<has_is_transparent<KE>::value>::type * = nullptr>
        std::pair<const_iterator, const_iterator> equal_range(const K &key) const { return m_ht.equal_range(key); }

        /**
         * @copydoc equal_range(const K& key, std::size_t precalculated_hash)
         */
        template<class K, class KE = KeyEqual, typename std::enable_if<has_is_transparent<KE>::value>::type * = nullptr>
        std::pair<const_iterator, const_iterator> equal_range(const K &key, std::size_t precalculated_hash) const {
            return m_ht.equal_range(key, precalculated_hash);
        }


        /*
         * Bucket interface
         */
        size_type bucket_count() const { return m_ht.bucket_count(); }

        size_type max_bucket_count() const { return m_ht.max_bucket_count(); }


        /*
         *  Hash policy
         */
        float load_factor() const { return m_ht.load_factor(); }

        float min_load_factor() const { return m_ht.min_load_factor(); }

        float max_load_factor() const { return m_ht.max_load_factor(); }

        /**
         * Set the `min_load_factor` to `ml`. When the `load_factor` of the map goes
         * below `min_load_factor` after some erase operations, the map will be
         * shrunk when an insertion occurs. The erase method itself never shrinks
         * the map.
         *
         * The default value of `min_load_factor` is 0.0f, the map never shrinks by default.
         */
        void min_load_factor(float ml) { m_ht.min_load_factor(ml); }

        void max_load_factor(float ml) { m_ht.max_load_factor(ml); }

        void rehash(size_type count) { m_ht.rehash(count); }

        void reserve(size_type count) { m_ht.reserve(count); }


        /*
         * Observers
         */
        hasher hash_function() const { return m_ht.hash_function(); }

        key_equal key_eq() const { return m_ht.key_eq(); }

        /*
         * Other
         */

        /**
         * Convert a const_iterator to an iterator.
         */
        iterator mutable_iterator(const_iterator pos) {
            return m_ht.mutable_iterator(pos);
        }

        friend bool operator==(const robin_map &lhs, const robin_map &rhs) {
            if (lhs.size() != rhs.size()) {
                return false;
            }

            for (const auto &element_lhs: lhs) {
                const auto it_element_rhs = rhs.find(element_lhs.first);
                if (it_element_rhs == rhs.cend() || element_lhs.second != it_element_rhs->second) {
                    return false;
                }
            }

            return true;
        }

        friend bool operator!=(const robin_map &lhs, const robin_map &rhs) {
            return !operator==(lhs, rhs);
        }

        friend void swap(robin_map &lhs, robin_map &rhs) {
            lhs.swap(rhs);
        }

    private:
        ht m_ht;
    };


/**
 * Same as `tsl::robin_map<Key, T, Hash, KeyEqual, Allocator, StoreHash, tsl::rh::prime_growth_policy>`.
 */
    template<class Key,
            class T,
            class Hash = std::hash<Key>,
            class KeyEqual = std::equal_to<Key>,
            class Allocator = std::allocator<std::pair<Key, T>>,
            bool StoreHash = false>
    using robin_pg_map = robin_map<Key, T, Hash, KeyEqual, Allocator, StoreHash, tsl::rh::prime_growth_policy>;

} // end namespace tsl
#endif
using tsl::robin_map;

using namespace std;
#define THREAD_COUNT 8
#define THREAD_COUNT_READ 4
#define PATCH_SIZE 100
#define MAXN 5000000
#define MAXE 2500000
#define RESULT_NUM 100
const unsigned int sizeTable[12] = {9, 99, 999, 9999, 99999, 999999, 9999999, 99999999, 999999999, 4294967295};
const char digits[256] =
        "0001020304050607080910111213141516171819"
        "2021222324252627282930313233343536373839"
        "4041424344454647484950515253545556575859"
        "6061626364656667686970717273747576777879"
        "8081828384858687888990919293949596979899";
typedef pair<uint64_t, uint64_t> pathPair;
//TODO ***********************
template <class T, class Alloc=std::allocator<T>>
class myVector
{
public:
    //vector的嵌套类型
    typedef T						value_type;
    typedef value_type*				iterator;
    typedef const value_type*		const_iterator;
    typedef value_type&				reference;
    typedef const T&                const_reference;
    typedef	size_t					size_type;
    typedef ptrdiff_t				difference_type; //表示两个迭代器之间的距离 ,c++内置定义 typedef int ptrdiff_t;
protected:
    iterator _start;					//数组的首元素
    iterator _end;					//目前使用空间的尾
    uint tail;
    value_type *data;
public:
    myVector() {
        data = (T *)malloc(MAXN * sizeof(T));
        _start = data;
        _end = _start;
        tail = 0;
    }//默认构造函数
//    myVector(size_type n, const T& value) {
//        _start = _alloc.allocate(n);						 //调用配置器函数分配内存
//        std::uninitialized_fill(_start, _start + n, value);  //调用c++内置函数进行初始化
//        _end = _end_of_storage = _start + n;                 //修改迭代器的指针
//    }//默认构造函数
//    myVector(iterator first, iterator last) {
//        _start=_alloc.allocate(last - first);  //分配空间
//        _end=_end_of_storage=std::uninitialized_copy(first, last, _start);
//    }
//    myVector(const myVector& v) {
//        size_type n= v.cend() - v.cbegin();
//        _start=_alloc.allocate(n);    //分配空间
//        _end = _end_of_storage = std::uninitialized_copy(v.cbegin(), v.cend(), _start);
//    }

    myVector& operator=(const myVector& rhs) {//赋值操作符函数
        if (this == &rhs)
            return *this;
        size_type n = rhs.cend() - rhs.cbegin();
        _start= (T *)malloc(n * sizeof(T));
        _end = std::uninitialized_copy(rhs.cbegin(), rhs.cend(), _start);
    }
    ~myVector() { _destroy(); }


    iterator begin() { return _start; }
    iterator end()	 { return _end; }
    const_iterator cbegin() const { return _start; }    //常量迭代器
    const_iterator cend() const { return _end; }

    size_type size()  { return size_type(_end - _start); }  //注意转换成size_t类型
    size_type capacity() { return size_type(_end - _start); }
    bool empty() const { return _start == _end; }
    void swap(myVector &other) {
        std::swap(_start, other._start);
        std::swap(_end, other._end);
    }

    reference front() { return *_start; }
    const_reference front() const { return *_start; }
    reference back() { return *(end() - 1); }
    reference operator[] (size_type n) { return *(begin() + n); }           //重载[],这样可以用a[n]进行访问元素
    const_reference top() {
        if (!empty())
            return data[tail];
        else return (T) NULL;
    }

    void push_back(const T& value) {
        data[tail++] = value;
        ++_end;
    }
    void pop_back() {
        --tail;
        --_end;               //这里要注意的是，删除尾部，要让_end先移动到最后一个元素
//        data[--tail] = (T) NULL;
    }
    void insert(iterator pos, const value_type & elem) {
        if (pos > end()) {
            *pos = elem;
            _end = pos + 1;
            return;
        }
        iterator p = _end;
        while (p != _start && p != pos){
            *p = *(p-1);
        }
        *pos = elem;
        ++_end;
    }
private:

    void _destroy() {
        free(data);
        _start = _end = NULL;
    }

};
struct NodeInfo{
    uint cur_idx;
    long dis;
    bool operator <(NodeInfo a) const  {  return dis < a.dis; }
    bool operator >(NodeInfo a) const  {  return dis > a.dis; }
};

struct Edge {
    unsigned int to;
    unsigned int weight;
};
struct Node {
    unsigned int l;
    unsigned int r;
};
struct Result {
    unsigned int id;
    double betweenness;
};
struct cmp_struct {
    bool operator () (Result &a, Result &b) {
        if (abs(a.betweenness - b.betweenness) > 0.0001)
            return a.betweenness > b.betweenness;
        else
            return a.id < b.id;
    }
};
bool cmp(Result a, Result b) {
    if (abs(a.betweenness - b.betweenness) > 0.0001)
        return a.betweenness > b.betweenness;
    else
        return a.id < b.id;
}

struct ThreadData {
    bool vis[MAXN];
    unsigned int S[MAXN];
    unsigned int P[MAXN];
    unsigned int pIdx[MAXE];
    unsigned int sigma[MAXN];
    uint64_t dis[MAXN];
    double delta[MAXN];
    double result[MAXN];
//     priority_queue<pathPair, vector<pathPair>, greater<pathPair>> Q;
    priority_queue<pathPair, myVector<pathPair>, greater<pathPair>> Q;
};
class MinHeap {
public:
    MinHeap() = default;

    void insert(Result res) {
        if (queue.size() < RESULT_NUM) {
            queue.emplace(res);
            minBc = queue.top().betweenness;
            return;
        }
        if (res.betweenness - minBc > 0) {
            queue.pop();
            queue.push(res);
            minBc = queue.top().betweenness;
        } else return;
    }

    Result* toArray() {
        for (int i = RESULT_NUM - 1; i >= 0; --i) {
            result[i] = queue.top();
            queue.pop();
        }
        return result;
    }
private:
    double minBc = UINT64_MAX;
    priority_queue<Result, vector<Result>, cmp_struct> queue;
    Result result[RESULT_NUM];
};

Result FinalResult[MAXN];
char Comma[MAXN][12];
Node G[MAXN];
Edge neighborsTable[MAXE];
unsigned int inDegreesStart[MAXN];
unsigned int nodeData[THREAD_COUNT_READ][2 * MAXE];
unsigned int index_Id[THREAD_COUNT_READ][2 * MAXE];
unsigned int moneyData[THREAD_COUNT_READ][MAXE];
unsigned int inDegree[THREAD_COUNT_READ][MAXN];
unsigned int outDegree[THREAD_COUNT_READ][MAXN];
unsigned int nodeIdxMulti[THREAD_COUNT_READ][3];
unsigned int overlap[MAXN];
unsigned int nodeCnt;
unsigned int patchCount, processedId;

mutex mtx;
//******************
ThreadData threadData[THREAD_COUNT];
struct timer{
    struct timeval startTime;
    struct timeval endTime;
    void setTime() {
        gettimeofday(&startTime, NULL);
    }
    void getTime() {
        gettimeofday(&endTime, NULL);
        cout << "Time:"<< int(1000 * (endTime.tv_sec - startTime.tv_sec) + (endTime.tv_usec - startTime.tv_usec) / 1000) << endl;
    }
};
void intToString(unsigned int value, char *dst) {
    unsigned int length = 0;
    for (;; ++length) {
        if (value <= sizeTable[length]) {
            ++length;
            break;
        }
    }
    unsigned int next = length - 1;
    while (value >= 100) {
        unsigned int i = (value % 100) * 2;
        value /= 100;
        dst[next] = digits[i + 1];
        dst[next - 1] = digits[i];
        next -= 2;
    }
    if (value < 10) {
        dst[next] = '0' + value;
    } else {
        unsigned int i = value * 2;
        dst[next] = digits[i + 1];
        dst[next - 1] = digits[i];
    }
//    dst[11] = length + 1;
    dst[length] = ',';
}

void addEdge(unsigned int u, unsigned int v, unsigned int m) {
    neighborsTable[G[u].r].to = v;
    neighborsTable[G[u].r].weight = m;
    ++G[u].r;
}
void idToIndex(unsigned int threadId, unsigned int *nodeData, robin_map<unsigned int, unsigned int> id_Index) {
    //id_to_index
    for (unsigned int idx = 0; idx < nodeIdxMulti[threadId][0]; idx += 2) {
        nodeData[idx] = id_Index[nodeData[idx]];
        nodeData[idx + 1] = id_Index[nodeData[idx + 1]];
        ++inDegree[threadId][nodeData[idx + 1]];
        ++outDegree[threadId][nodeData[idx]];
    }
}
void merge(unsigned int *A, unsigned int m, const unsigned int *B, unsigned int n) {
    int pa = m - 1, pb = n - 1;
    unsigned int tail = m + n - 1;
    while (pa >= 0 && pb >= 0) {
        if (A[pa] > B[pb])
            A[tail--] = A[pa--];
        else
            A[tail--] = B[pb--];
    }
    while (pb >= 0) {
        A[tail--] = B[pb--];
    }
}
void sortThread(char *buf, const char *end, unsigned int *nodeData, unsigned int *moneyData,
                unsigned int *index_Id, unsigned int threadId) {
    unsigned int nodeIdx = 0, nodeIdx2 = 0, nodeIdx3 = 0;
    while (buf < end) {
        unsigned int from = 0, to = 0;
        unsigned int money = 0;
        while ((*buf) & 0x10) {
            from *= 10;
            from += (*buf) & 0x0f;
            buf++;
        }
        ++buf;
        while ((*buf) & 0x10) {
            to *= 10;
            to += (*buf) & 0x0f;
            buf++;
        }
        ++buf;
        while ((*buf) & 0x10) {
            money *= 10;
            money += (*buf) & 0x0f;
            buf++;
        }
        if (*buf == '\r') {
            ++buf;
        }
        ++buf;
        if (money == 0) continue;
        nodeData[nodeIdx++] = from;
        nodeData[nodeIdx++] = to;
        moneyData[nodeIdx2++] = money;
        index_Id[nodeIdx3++] = from;
        index_Id[nodeIdx3++] = to;
    }
    sort(index_Id, index_Id + nodeIdx3);
    nodeIdxMulti[threadId][0] = nodeIdx;
    nodeIdxMulti[threadId][1] = nodeIdx2;
    nodeIdxMulti[threadId][2] = nodeIdx3;
}

void readData(char *inputFilePath) {
    robin_map<unsigned int, unsigned int> id_Index;
    int fd = open(inputFilePath, O_RDONLY);
    int fileLen = lseek(fd, 0, SEEK_END);
    char *buf = (char *) mmap(NULL, fileLen, PROT_READ, MAP_PRIVATE, fd, 0);
    close(fd);
    char *start[] = {buf, buf + fileLen / THREAD_COUNT_READ, buf + fileLen / THREAD_COUNT_READ * 2,
                     buf + fileLen / THREAD_COUNT_READ * 3, buf + fileLen};
    for (unsigned int i = 1; i < THREAD_COUNT_READ; i++) {
        while (*(start[i]++) != '\n');
    }
    thread t[THREAD_COUNT_READ];
    for (unsigned int i = 0; i < THREAD_COUNT_READ; i++) {
        t[i] = thread(sortThread, start[i], start[i + 1], nodeData[i], moneyData[i], index_Id[i], i);
    }
    for (unsigned int i = 0; i < THREAD_COUNT_READ; i++)
        t[i].join();
    merge(index_Id[0], nodeIdxMulti[0][2], index_Id[1], nodeIdxMulti[1][2]);
    merge(index_Id[2], nodeIdxMulti[2][2], index_Id[3], nodeIdxMulti[3][2]);
    merge(index_Id[0], nodeIdxMulti[0][2] + nodeIdxMulti[1][2], index_Id[2], nodeIdxMulti[2][2] + nodeIdxMulti[3][2]);
    unsigned int nodeIdx3 = 0;
    for (unsigned int i = 0; i < THREAD_COUNT_READ; i++)
        nodeIdx3 += nodeIdxMulti[i][2];
    nodeCnt = unique(index_Id[0], index_Id[0] + nodeIdx3) - index_Id[0];
    id_Index.rehash(1024 * 1024);
    id_Index.reserve(nodeCnt);
    for (unsigned int idx = 0; idx < nodeCnt; idx++) {
        id_Index[index_Id[0][idx]] = idx;
        intToString(index_Id[0][idx], Comma[idx]);
    }
    for (unsigned int i = 0; i < THREAD_COUNT_READ; i++)
        t[i] = thread(idToIndex, i, nodeData[i], id_Index);
    for (unsigned int i = 0; i < THREAD_COUNT_READ; i++)
        t[i].join();
    for (unsigned int idx = 0; idx < nodeCnt; idx++) {
        outDegree[0][idx] += outDegree[1][idx] + outDegree[2][idx] + outDegree[3][idx];
        inDegree[0][idx] += inDegree[1][idx] + inDegree[2][idx] + inDegree[3][idx];
    }
    G[0].l = 0;
    G[0].r = 0;
    for (unsigned int idx = 1; idx < nodeCnt; idx++) {
        G[idx].l = G[idx - 1].l + outDegree[0][idx - 1];
        G[idx].r = G[idx].l;
        inDegreesStart[idx]=inDegreesStart[idx-1]+inDegree[0][idx-1];
    }
    for (unsigned int i = 0; i < THREAD_COUNT_READ; i++) {
        for (unsigned int idx = 0, idx2 = 0; idx < nodeIdxMulti[i][0]; idx += 2) {
            addEdge(nodeData[i][idx], nodeData[i][idx + 1], moneyData[i][idx2++]);
        }
    }
    patchCount = nodeCnt / PATCH_SIZE;
}

void searchResult(unsigned int threadId) {
    bool *vis = threadData[threadId].vis;
    unsigned int *S = threadData[threadId].S;
    unsigned int *P = threadData[threadId].P;
    unsigned int *pIdx =threadData[threadId].pIdx;
    unsigned int *sigma = threadData[threadId].sigma;
    uint64_t *dis = threadData[threadId].dis;
    double *delta = threadData[threadId].delta;
    double *result = threadData[threadId].result;
    priority_queue<pathPair, myVector<pathPair>, greater<pathPair>> &Q = threadData[threadId].Q;
    fill(dis,dis+nodeCnt,UINT64_MAX);
    while (true) {
        mtx.lock();
        unsigned int patchId = processedId++;
        mtx.unlock();
//        if(patchId % 20 == 0){
//            cout << "solved patch:" << patchId << "/" << patchCount << endl;
//        }
        if (patchId >= patchCount)
            return;
        unsigned int startId = patchId * PATCH_SIZE, endId, sIdx = 0, idx;
        if (patchId == patchCount - 1)
            endId = nodeCnt;
        else
            endId = (patchId + 1) * PATCH_SIZE;
        for (unsigned int start = startId; start < endId; start++) {
            if(outDegree[0][start]==0) continue;
//            if(outDegree[0][start] == 1 && inDegree[0][start] == 0)
//                for (idx = G[start].l; idx < G[start].r; idx++)
//                    if(outDegree[0][neighborsTable[idx].to]==1)
//                        cout << "Node:" << neighborsTable[idx].to << "  InDgree:" << inDegree[0][neighborsTable[idx].to]
//                             << "  OutDgree:" << outDegree[0][neighborsTable[idx].to] << endl;
            if(overlap[start] == 0)
                continue;
            sigma[start] = 1; dis[start]=0;
            Q.push(make_pair(0, start));
            while (!Q.empty()) {
                unsigned int u = Q.top().second;
                Q.pop();
                if (vis[u]) continue;
                vis[u] = true;
                S[sIdx++] = u;
                for (idx = G[u].l; idx < G[u].r; idx++) {
                    unsigned int &v = neighborsTable[idx].to;
                    unsigned int &weight = neighborsTable[idx].weight;
                    if (dis[u] + weight < dis[v]) {
                        dis[v] = dis[u] + weight;
                        sigma[v] = sigma[u];
                        pIdx[v]=inDegreesStart[v];
                        P[pIdx[v]++]=u;
                        if (!vis[v])
                            Q.push(make_pair(dis[v], v));
                    } else if (dis[u] + weight == dis[v]) {
                        if(sigma[u]) {
                            sigma[v] += sigma[u];
                            P[pIdx[v]++] = u;
                        }
                    }
                }
            }
            while (sIdx > 0) {
                unsigned int &v = S[--sIdx];
                double caf = (1 + delta[v]) / sigma[v];
                for (idx = inDegreesStart[v]; idx < pIdx[v]; idx++) {
                    delta[P[idx]] += caf * sigma[P[idx]];
                }

                if (v != start) {
                    result[v] += delta[v]*(overlap[start]);
                    result[start]+=overlap[start]-1;
                }
                delta[v] = 0;sigma[v] = 0;
                vis[v] = false;
                dis[v] = UINT64_MAX;
            }
        }
    }
}


static void writeResult(char *outputFilePath) {
    MinHeap max_queue;
    //合并线程结果
    for (unsigned int node = 0; node < nodeCnt; node++) {
        FinalResult[node].id = node;
        for (unsigned int t = 0; t < THREAD_COUNT; t++) {
            FinalResult[node].betweenness += threadData[t].result[node];
        }
        max_queue.insert(FinalResult[node]);
    }
    ofstream fout(outputFilePath);
    Result *res = max_queue.toArray();
    for (unsigned int idx = 0; idx < RESULT_NUM; idx++) {
        string s = Comma[res[idx].id];
        fout << s;
        fout << fixed << setprecision(3) << res[idx].betweenness << endl;
    }
    fout.close();
}
void preProcessing(){
    for(uint idx = 0; idx < nodeCnt; idx++){
        if(inDegree[0][idx] == 0 && outDegree[0][idx] == 1){
            overlap[neighborsTable[G[idx].l].to] += 1;
        }else{
            overlap[idx] += 1;
        }
    }
}
/*************************** tarjan ***************************************/
vector<vector<unsigned int>> tarRes;
void tarjan() {
    cout << "tarjan.." << endl;
    clock_t start = clock();
    int visitTime = 0;
    unordered_map<unsigned int, int> preorder;
    unordered_map<unsigned int, int> lowlink;
    unordered_map<unsigned int, bool> sccFound;
    stack<unsigned int> sccQueue;
    for (int node = 0; node < nodeCnt; ++node) {
        stack<unsigned int> queue;
        if (!sccFound.count(node)) {
            queue.push(node);
            while (!queue.empty()) {
                uint v = queue.top();
                if (!preorder.count(v))
                    preorder[v] = ++visitTime;
                bool done = true;
                for (uint index = G[v].l; index < G[v].r; ++index) {
                    uint w = neighborsTable[index].to;
                    if (!preorder.count(w)) {
                        queue.push(w);
                        done = false;
                        break;
                    }
                }
                if (done) {
                    lowlink[v] = preorder[v];
                    for (uint index = G[v].l; index < G[v].r; ++index) {
                        uint w = neighborsTable[index].to;
                        if (!sccFound.count(w)) {
                            if (preorder[w] > preorder[v]) {
                                lowlink[v] = min(lowlink[v], lowlink[w]);
                            } else {
                                lowlink[v] = min(lowlink[v], preorder[w]);
                            }
                        }
                    }
                    queue.pop();
                    if (lowlink[v] == preorder[v]) {
                        sccFound[v] = true;
                        vector<uint> scc;
                        scc.emplace_back(v);
                        while (!sccQueue.empty() && preorder[sccQueue.top()] > preorder[v]) {
                            uint k = sccQueue.top();
                            sccQueue.pop();
                            sccFound[k] = true;
                            scc.emplace_back(k);
                        }
                        if (scc.size() > 2)
                            tarRes.emplace_back(scc);
                    } else
                        sccQueue.push(v);

                }
            }
        }
    }
    clock_t end = clock();
    cout << "tarjan time: " << (double)(end - start) / CLOCKS_PER_SEC << endl;
    cout << "tarjan_result: " << tarRes.size();

}
int main() {
    nice(-20);
    char *inputFilePath = (char *) "/home/data13.txt";
    char *outputFilePath = (char *) "/codeCraftFinal/res13.txt";
//    char *inputFilePath = (char *) "/data/test_data.txt";
//    char *outputFilePath = (char *) "/projects/student/result.txt";
    readData(inputFilePath);
    preProcessing();
    //cout << "nodeCnt:" << nodeCnt << endl;
    std::thread threads[THREAD_COUNT - 1];
    int t;
    for (t = 0; t < THREAD_COUNT - 1; t++) {
        threads[t] = std::thread(searchResult, t);
    }
    searchResult(t);
    for (t = 0; t < THREAD_COUNT - 1; t++) {
        threads[t].join();
    }
    writeResult(outputFilePath);

    exit(0);
}