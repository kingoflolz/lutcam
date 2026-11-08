//! Functional model of the bounded, sorted child-list arena.

use std::cmp::Ordering;
use std::collections::BTreeSet;
use std::fmt;
use std::sync::Arc;

pub const NULL_ID: isize = -1;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ChildArenaError {
    InvalidConfiguration(&'static str),
    Full,
    InvalidId(isize),
    NotInitialized(usize),
    AlreadyInitialized(usize),
    Invariant(String),
}

impl fmt::Display for ChildArenaError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidConfiguration(message) => f.write_str(message),
            Self::Full => f.write_str("child arena has no free entries"),
            Self::InvalidId(id) => write!(f, "{id} is not a valid child-list ID"),
            Self::NotInitialized(id) => write!(f, "{id} is not an initialized child-list head"),
            Self::AlreadyInitialized(id) => write!(f, "{id} is already initialized"),
            Self::Invariant(message) => write!(f, "child-arena invariant failed: {message}"),
        }
    }
}

impl std::error::Error for ChildArenaError {}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChildArenaEntry<F> {
    pub feature: Option<F>,
    pub next: Option<usize>,
    pub len: usize,
}

impl<F> Default for ChildArenaEntry<F> {
    fn default() -> Self {
        Self {
            feature: None,
            next: None,
            len: 0,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChildArenaReadResult<F> {
    pub features: Vec<F>,
}

type Compare<F> = Arc<dyn Fn(&F, &F) -> Ordering + Send + Sync>;

pub struct ChildArenaReferenceModel<F> {
    pub entry_count: usize,
    pub max_list_len: usize,
    pub equal_key_service_min: bool,
    pub entries: Vec<ChildArenaEntry<F>>,
    pub free_head: Option<usize>,
    pub free_count: usize,
    free_ids: BTreeSet<usize>,
    valid_heads: BTreeSet<usize>,
    compare: Compare<F>,
}

impl<F: Ord + Clone + Send + Sync + 'static> ChildArenaReferenceModel<F> {
    pub fn new(entry_count: usize, max_list_len: usize) -> Result<Self, ChildArenaError> {
        Self::with_comparator(entry_count, max_list_len, false, |a, b| a.cmp(b))
    }
}

impl<F: Clone + Send + Sync + 'static> ChildArenaReferenceModel<F> {
    pub fn with_sort_key<K, S>(
        entry_count: usize,
        max_list_len: usize,
        equal_key_service_min: bool,
        sort_key: S,
    ) -> Result<Self, ChildArenaError>
    where
        K: Ord,
        S: Fn(&F) -> K + Send + Sync + 'static,
    {
        Self::with_comparator(
            entry_count,
            max_list_len,
            equal_key_service_min,
            move |a, b| sort_key(a).cmp(&sort_key(b)),
        )
    }

    pub fn with_comparator<C>(
        entry_count: usize,
        max_list_len: usize,
        equal_key_service_min: bool,
        compare: C,
    ) -> Result<Self, ChildArenaError>
    where
        C: Fn(&F, &F) -> Ordering + Send + Sync + 'static,
    {
        if entry_count == 0 {
            return Err(ChildArenaError::InvalidConfiguration(
                "entry_count must be positive",
            ));
        }
        if max_list_len == 0 {
            return Err(ChildArenaError::InvalidConfiguration(
                "max_list_len must be positive",
            ));
        }
        let mut entries: Vec<_> = (0..entry_count)
            .map(|_| ChildArenaEntry::default())
            .collect();
        for (index, entry) in entries.iter_mut().enumerate() {
            entry.next = (index + 1 < entry_count).then_some(index + 1);
        }
        Ok(Self {
            entry_count,
            max_list_len,
            equal_key_service_min,
            entries,
            free_head: Some(0),
            free_count: entry_count,
            free_ids: (0..entry_count).collect(),
            valid_heads: BTreeSet::new(),
            compare: Arc::new(compare),
        })
    }

    pub fn valid_heads(&self) -> &BTreeSet<usize> {
        &self.valid_heads
    }

    /// Allocate a stable, uninitialized list head.
    pub fn allocate(&mut self) -> Result<usize, ChildArenaError> {
        let node = self.pop_free()?;
        self.entries[node] = ChildArenaEntry::default();
        self.valid_heads.insert(node);
        Ok(node)
    }

    pub fn init(&mut self, head: usize, feature: F) -> Result<(), ChildArenaError> {
        self.require_head(head)?;
        if self.entries[head].len != 0 {
            return Err(ChildArenaError::AlreadyInitialized(head));
        }
        self.entries[head] = ChildArenaEntry {
            feature: Some(feature),
            next: None,
            len: 1,
        };
        Ok(())
    }

    pub fn add(&mut self, head: usize, feature: F) -> Result<(), ChildArenaError> {
        self.require_initialized(head)?;
        if self.entries[head].len < self.max_list_len {
            return self.add_non_full(head, feature);
        }

        let head_feature = self.entry_feature(head)?;
        if (self.compare)(&feature, head_feature) != Ordering::Greater {
            return Ok(());
        }
        if self.max_list_len == 1 {
            self.entries[head].feature = Some(feature);
            return Ok(());
        }

        let reuse = self.entries[head]
            .next
            .ok_or_else(|| ChildArenaError::Invariant("full list has no second node".into()))?;
        let reuse_feature = self.entry_feature(reuse)?.clone();
        let after_reuse = self.entries[reuse].next;
        if (self.compare)(&feature, &reuse_feature) != Ordering::Greater {
            self.entries[head].feature = Some(feature);
            self.entries[head].next = Some(reuse);
            self.entries[reuse].next = after_reuse;
            return Ok(());
        }

        self.entries[head].feature = Some(reuse_feature);
        self.entries[head].next = after_reuse;
        let mut prev = head;
        let mut current = self.entries[head].next;
        while let Some(node) = current {
            if !self.scan_past_equal(self.entry_feature(node)?, &feature) {
                break;
            }
            prev = node;
            current = self.entries[node].next;
        }
        self.entries[reuse].feature = Some(feature);
        self.entries[reuse].next = current;
        self.entries[prev].next = Some(reuse);
        Ok(())
    }

    pub fn merge(
        &mut self,
        left: Option<usize>,
        right: Option<usize>,
    ) -> Result<Option<usize>, ChildArenaError> {
        if left.is_none() || right.is_none() || left == right {
            if let Some(head) = left {
                self.require_head(head)?;
            }
            if let Some(head) = right {
                self.require_head(head)?;
            }
            return Ok(left.or(right));
        }
        let (left, right) = (left.unwrap(), right.unwrap());
        self.require_initialized(left)?;
        self.require_initialized(right)?;
        let canonical = left.min(right);
        let old = left.max(right);
        let canonical_nodes = self.list_nodes(canonical)?;
        let old_nodes = self.list_nodes(old)?;
        let canonical_features: Vec<F> = canonical_nodes
            .iter()
            .map(|&n| self.entry_feature(n).cloned())
            .collect::<Result<_, _>>()?;
        let old_features: Vec<F> = old_nodes
            .iter()
            .map(|&n| self.entry_feature(n).cloned())
            .collect::<Result<_, _>>()?;

        let old_below_head = match (old_features.last(), canonical_features.first()) {
            (Some(old_tail), Some(canonical_head)) => {
                let order = (self.compare)(old_tail, canonical_head);
                if self.equal_key_service_min {
                    order == Ordering::Less
                } else {
                    order != Ordering::Greater
                }
            }
            _ => false,
        };
        if self.entries[canonical].len == self.max_list_len && old_below_head {
            self.valid_heads.remove(&old);
            self.free_nodes(&old_nodes)?;
            return Ok(Some(canonical));
        }

        let merged = self.stable_merge(&canonical_features, &old_features);
        let target_len = merged.len().min(self.max_list_len);
        let retained_features = merged[merged.len() - target_len..].to_vec();
        let mut all_nodes = canonical_nodes.clone();
        all_nodes.extend(&old_nodes);
        let mut retained_nodes = vec![canonical];
        retained_nodes.extend(all_nodes.iter().copied().filter(|&n| n != canonical));
        retained_nodes.truncate(target_len);
        let retained_set: BTreeSet<_> = retained_nodes.iter().copied().collect();
        let freed: Vec<_> = all_nodes
            .into_iter()
            .filter(|n| !retained_set.contains(n))
            .collect();
        self.valid_heads.remove(&old);
        for (index, (&node, feature)) in retained_nodes.iter().zip(retained_features).enumerate() {
            self.entries[node].feature = Some(feature);
            self.entries[node].next = retained_nodes.get(index + 1).copied();
            self.entries[node].len = if index == 0 { target_len } else { 0 };
        }
        self.free_nodes(&freed)?;
        Ok(Some(canonical))
    }

    /// Read and free a list.  Hardware only emits lists that reached the
    /// configured maximum length; shorter lists are returned empty.
    pub fn read(&mut self, head: usize) -> Result<ChildArenaReadResult<F>, ChildArenaError> {
        self.read_impl(head, false)
    }

    /// Read and free every feature in a nonempty list.  This is a reference-
    /// only inspection path used to validate candidates independently of the
    /// hardware's bounded-list output policy.
    pub fn read_all(&mut self, head: usize) -> Result<ChildArenaReadResult<F>, ChildArenaError> {
        self.read_impl(head, true)
    }

    fn read_impl(
        &mut self,
        head: usize,
        emit_short_list: bool,
    ) -> Result<ChildArenaReadResult<F>, ChildArenaError> {
        self.require_initialized(head)?;
        let nodes = self.list_nodes(head)?;
        let emit = emit_short_list || nodes.len() >= self.max_list_len;
        let result = ChildArenaReadResult {
            features: if emit {
                nodes
                    .iter()
                    .map(|&n| self.entry_feature(n).cloned())
                    .collect::<Result<_, _>>()?
            } else {
                Vec::new()
            },
        };
        self.valid_heads.remove(&head);
        self.free_nodes(&nodes)?;
        Ok(result)
    }

    pub fn features(&self, head: usize) -> Result<Vec<F>, ChildArenaError> {
        self.require_initialized(head)?;
        self.list_nodes(head)?
            .iter()
            .map(|&n| self.entry_feature(n).cloned())
            .collect()
    }

    pub fn check_invariants(&self) -> Result<(), ChildArenaError> {
        let mut free_nodes = BTreeSet::new();
        let mut node = self.free_head;
        while let Some(index) = node {
            self.require_index(index as isize)?;
            if !free_nodes.insert(index) {
                return Err(ChildArenaError::Invariant(
                    "free list contains a cycle".into(),
                ));
            }
            node = self.entries[index].next;
        }
        if free_nodes != self.free_ids {
            return Err(ChildArenaError::Invariant(
                "free-list links and ID set disagree".into(),
            ));
        }
        if free_nodes.len() != self.free_count {
            return Err(ChildArenaError::Invariant(
                "free_count is inconsistent".into(),
            ));
        }

        let mut live = BTreeSet::new();
        let mut reserved = BTreeSet::new();
        for &head in &self.valid_heads {
            if free_nodes.contains(&head) {
                return Err(ChildArenaError::Invariant("valid head is free".into()));
            }
            if self.entries[head].len == 0 {
                if self.entries[head].next.is_some() || self.entries[head].feature.is_some() {
                    return Err(ChildArenaError::Invariant(
                        "empty head carries state".into(),
                    ));
                }
                reserved.insert(head);
                continue;
            }
            let nodes = self.list_nodes(head)?;
            if nodes.len() != self.entries[head].len || nodes.len() > self.max_list_len {
                return Err(ChildArenaError::Invariant(
                    "stored list length is inconsistent".into(),
                ));
            }
            for pair in nodes.windows(2) {
                if (self.compare)(self.entry_feature(pair[0])?, self.entry_feature(pair[1])?)
                    == Ordering::Greater
                {
                    return Err(ChildArenaError::Invariant("list is not sorted".into()));
                }
            }
            for node in nodes {
                if !live.insert(node) {
                    return Err(ChildArenaError::Invariant(
                        "node appears in multiple lists".into(),
                    ));
                }
            }
        }
        if live.iter().any(|n| free_nodes.contains(n))
            || reserved
                .iter()
                .any(|n| live.contains(n) || free_nodes.contains(n))
        {
            return Err(ChildArenaError::Invariant("arena sets overlap".into()));
        }
        if live.len() + reserved.len() + free_nodes.len() != self.entry_count {
            return Err(ChildArenaError::Invariant(
                "some nodes are unaccounted for".into(),
            ));
        }
        Ok(())
    }

    fn add_non_full(&mut self, head: usize, feature: F) -> Result<(), ChildArenaError> {
        let node = self.pop_free()?;
        let head_feature = self.entry_feature(head)?.clone();
        match (self.compare)(&feature, &head_feature) {
            Ordering::Less => {
                self.entries[node] = ChildArenaEntry {
                    feature: self.entries[head].feature.clone(),
                    next: self.entries[head].next,
                    len: 0,
                };
                self.entries[head].feature = Some(feature);
                self.entries[head].next = Some(node);
            }
            Ordering::Equal => {
                self.entries[node] = ChildArenaEntry {
                    feature: Some(feature),
                    next: self.entries[head].next,
                    len: 0,
                };
                self.entries[head].next = Some(node);
            }
            Ordering::Greater => {
                let mut prev = head;
                let mut current = self.entries[head].next;
                while let Some(candidate) = current {
                    if !self.scan_past_equal(self.entry_feature(candidate)?, &feature) {
                        break;
                    }
                    prev = candidate;
                    current = self.entries[candidate].next;
                }
                self.entries[node] = ChildArenaEntry {
                    feature: Some(feature),
                    next: current,
                    len: 0,
                };
                self.entries[prev].next = Some(node);
            }
        }
        self.entries[head].len += 1;
        Ok(())
    }

    fn stable_merge(&self, canonical: &[F], old: &[F]) -> Vec<F> {
        let mut merged = Vec::with_capacity(canonical.len() + old.len());
        let (mut ci, mut oi) = (0, 0);
        let mut drop_remaining = (canonical.len() + old.len()).saturating_sub(self.max_list_len);
        while ci < canonical.len() || oi < old.len() {
            let choose_canonical = if oi == old.len() {
                true
            } else if ci == canonical.len() {
                false
            } else {
                match (self.compare)(&canonical[ci], &old[oi]) {
                    Ordering::Less => true,
                    Ordering::Greater => false,
                    Ordering::Equal if !self.equal_key_service_min => true,
                    Ordering::Equal if drop_remaining != 0 => false,
                    Ordering::Equal => canonical.len() - ci >= old.len() - oi,
                }
            };
            if choose_canonical {
                merged.push(canonical[ci].clone());
                ci += 1;
            } else {
                merged.push(old[oi].clone());
                oi += 1;
            }
            drop_remaining = drop_remaining.saturating_sub(1);
        }
        merged
    }

    fn scan_past_equal(&self, existing: &F, candidate: &F) -> bool {
        let order = (self.compare)(existing, candidate);
        if self.equal_key_service_min {
            order == Ordering::Less
        } else {
            order != Ordering::Greater
        }
    }

    fn list_nodes(&self, head: usize) -> Result<Vec<usize>, ChildArenaError> {
        self.require_head(head)?;
        let mut nodes = Vec::new();
        let mut seen = BTreeSet::new();
        let mut node = Some(head);
        let remaining = self.entries[head].len;
        if remaining == 0 {
            if self.entries[head].next.is_some() {
                return Err(ChildArenaError::Invariant(
                    "empty list has successor".into(),
                ));
            }
            return Ok(nodes);
        }
        for _ in 0..remaining {
            let index = node.ok_or_else(|| {
                ChildArenaError::Invariant("list ended before stored length".into())
            })?;
            self.require_index(index as isize)?;
            if !seen.insert(index) {
                return Err(ChildArenaError::Invariant("list contains a cycle".into()));
            }
            if self.free_ids.contains(&index) {
                return Err(ChildArenaError::Invariant(
                    "list contains a free node".into(),
                ));
            }
            nodes.push(index);
            node = self.entries[index].next;
        }
        if node.is_some() {
            return Err(ChildArenaError::Invariant(
                "list exceeds stored length".into(),
            ));
        }
        Ok(nodes)
    }

    fn pop_free(&mut self) -> Result<usize, ChildArenaError> {
        let node = self.free_head.ok_or(ChildArenaError::Full)?;
        self.free_head = self.entries[node].next;
        self.free_count -= 1;
        self.free_ids.remove(&node);
        self.entries[node].next = None;
        Ok(node)
    }

    fn free_nodes(&mut self, nodes: &[usize]) -> Result<(), ChildArenaError> {
        if nodes.is_empty() {
            return Ok(());
        }
        for &node in nodes {
            self.require_index(node as isize)?;
            if self.free_ids.contains(&node) {
                return Err(ChildArenaError::Invariant("node freed twice".into()));
            }
            self.entries[node] = ChildArenaEntry::default();
            self.free_ids.insert(node);
        }
        for (index, &node) in nodes.iter().enumerate() {
            self.entries[node].next = nodes.get(index + 1).copied().or(self.free_head);
        }
        self.free_head = Some(nodes[0]);
        self.free_count += nodes.len();
        Ok(())
    }

    fn entry_feature(&self, node: usize) -> Result<&F, ChildArenaError> {
        self.entries[node]
            .feature
            .as_ref()
            .ok_or_else(|| ChildArenaError::Invariant("live entry has no feature".into()))
    }
    fn require_head(&self, head: usize) -> Result<(), ChildArenaError> {
        self.require_index(head as isize)?;
        if !self.valid_heads.contains(&head) {
            return Err(ChildArenaError::InvalidId(head as isize));
        }
        Ok(())
    }
    fn require_initialized(&self, head: usize) -> Result<(), ChildArenaError> {
        self.require_head(head)?;
        if self.entries[head].len == 0 {
            return Err(ChildArenaError::NotInitialized(head));
        }
        Ok(())
    }
    fn require_index(&self, index: isize) -> Result<(), ChildArenaError> {
        if index < 0 || index as usize >= self.entry_count {
            Err(ChildArenaError::InvalidId(index))
        } else {
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keeps_largest_sorted() {
        let mut arena = ChildArenaReferenceModel::<i32>::new(16, 4).unwrap();
        let head = arena.allocate().unwrap();
        arena.init(head, 4).unwrap();
        for value in [1, 9, 3, 8, 7, 2] {
            arena.add(head, value).unwrap();
        }
        assert_eq!(arena.features(head).unwrap(), vec![4, 7, 8, 9]);
        arena.check_invariants().unwrap();
    }

    #[test]
    fn merge_uses_lower_id_and_read_frees() {
        let mut arena = ChildArenaReferenceModel::<i32>::new(12, 3).unwrap();
        let a = arena.allocate().unwrap();
        let b = arena.allocate().unwrap();
        arena.init(a, 1).unwrap();
        arena.add(a, 8).unwrap();
        arena.init(b, 3).unwrap();
        arena.add(b, 9).unwrap();
        assert_eq!(arena.merge(Some(a), Some(b)).unwrap(), Some(a));
        assert_eq!(arena.features(a).unwrap(), vec![3, 8, 9]);
        let read = arena.read(a).unwrap();
        assert_eq!(read.features, vec![3, 8, 9]);
        assert_eq!(arena.free_count, 12);
    }

    #[test]
    fn short_read_is_suppressed() {
        let mut arena = ChildArenaReferenceModel::<i32>::new(4, 3).unwrap();
        let head = arena.allocate().unwrap();
        arena.init(head, 1).unwrap();
        arena.add(head, 2).unwrap();
        assert!(arena.read(head).unwrap().features.is_empty());
    }

    #[test]
    fn read_all_returns_and_frees_a_short_list() {
        let mut arena = ChildArenaReferenceModel::<i32>::new(4, 3).unwrap();
        let head = arena.allocate().unwrap();
        arena.init(head, 1).unwrap();
        arena.add(head, 2).unwrap();
        let read = arena.read_all(head).unwrap();
        assert_eq!(read.features, vec![1, 2]);
        assert_eq!(arena.free_count, 4);
    }
}
