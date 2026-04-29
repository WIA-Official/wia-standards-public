# 제5장: API 인터페이스

## 개요

> **弘益人間 (홍익인간)** - 널리 인간을 이롭게 하라

WIA-LANG-010 표준은 다양한 프로그래밍 언어를 위한 API를 제공합니다. 이 장에서는 TypeScript/JavaScript, Python, Java API를 중심으로 언어 데이터를 생성, 조작, 검증, 쿼리하는 방법을 설명합니다.

## 1. TypeScript/JavaScript API

### 1.1 설치 및 초기화

```bash
# npm을 통한 설치
npm install @wia/lang-010

# yarn을 통한 설치
yarn add @wia/lang-010
```

```typescript
import {
  LinguisticData,
  Corpus,
  Document,
  Sentence,
  Token,
  Annotation,
  AnnotationLayer
} from '@wia/lang-010';

// 새로운 언어 데이터 생성
const data = new LinguisticData({
  version: '1.0.0',
  metadata: {
    language: {
      iso639_3: 'kor',
      name: 'Korean',
      script: 'Hangul'
    },
    title: 'My Korean Corpus',
    creator: {
      name: 'Research Lab',
      email: 'lab@example.com'
    },
    license: {
      type: 'CC-BY-SA-4.0',
      url: 'https://creativecommons.org/licenses/by-sa/4.0/'
    }
  }
});
```

### 1.2 코퍼스 조작

```typescript
// 코퍼스 생성
const corpus = data.createCorpus();

// 문서 추가
const doc = corpus.addDocument({
  id: 'doc001',
  metadata: {
    source: '신문사 A',
    date: '2025-01-20',
    genre: 'news',
    domain: 'technology'
  }
});

// 문장 추가
const sentence = doc.addSentence({
  id: 's001',
  text: '인공지능 기술이 빠르게 발전하고 있다.'
});

// 토큰 추가
const tokens = [
  sentence.addToken({ id: 't001', form: '인공지능', start: 0, end: 4 }),
  sentence.addToken({ id: 't002', form: '기술이', start: 5, end: 8 }),
  sentence.addToken({ id: 't003', form: '빠르게', start: 9, end: 12 }),
  sentence.addToken({ id: 't004', form: '발전하고', start: 13, end: 17 }),
  sentence.addToken({ id: 't005', form: '있다', start: 18, end: 20 }),
  sentence.addToken({ id: 't006', form: '.', start: 20, end: 21 })
];

// 형태소 분석 추가
tokens[0].addMorphemes([
  { form: '인공', lemma: '인공', tag: 'NNG', gloss: 'artificial' },
  { form: '지능', lemma: '지능', tag: 'NNG', gloss: 'intelligence' }
]);

tokens[1].addMorphemes([
  { form: '기술', lemma: '기술', tag: 'NNG', gloss: 'technology' },
  { form: '이', lemma: '이', tag: 'JKS', gloss: 'NOM' }
]);

// 품사 태깅
tokens[0].setPos('NOUN', 'NNG');
tokens[1].setPos('NOUN', 'NNG+JKS');
tokens[2].setPos('ADV', 'VA+EC');
tokens[3].setPos('VERB', 'NNG+VV+EC');
tokens[4].setPos('AUX', 'VX+EF');
tokens[5].setPos('PUNCT', 'SF');

// 의존 관계 추가
tokens[0].setDependency({ head: tokens[1], relation: 'nmod' });
tokens[1].setDependency({ head: tokens[3], relation: 'nsubj' });
tokens[2].setDependency({ head: tokens[3], relation: 'advmod' });
tokens[3].setDependency({ head: tokens[4], relation: 'aux' });
tokens[4].setDependency({ head: null, relation: 'root' });
tokens[5].setDependency({ head: tokens[4], relation: 'punct' });

// 개체명 추가
sentence.addEntity({
  id: 'e001',
  type: 'TECHNOLOGY',
  start: 0,
  end: 4,
  text: '인공지능',
  tokens: [tokens[0]],
  confidence: 0.95
});
```

### 1.3 쿼리 및 검색

```typescript
// 문장 검색
const sentences = corpus.findSentences({
  contains: '인공지능',
  minLength: 5,
  maxLength: 20
});

// 토큰 검색
const nounsWithJKS = corpus.findTokens({
  pos: 'NOUN',
  hasMorpheme: (m) => m.tag === 'JKS'
});

// 의존 관계 검색
const nsubjRelations = corpus.findDependencies({
  relation: 'nsubj'
});

// 개체명 검색
const techEntities = corpus.findEntities({
  type: 'TECHNOLOGY',
  minConfidence: 0.9
});

// 복잡한 쿼리
const complexQuery = corpus.query({
  sentence: {
    hasEntity: { type: 'PERSON' },
    hasToken: { pos: 'VERB' }
  },
  token: {
    form: /^[가-힣]+$/,  // 한글만
    lemma: { $in: ['하다', '되다', '이다'] }
  }
});

// XPath 스타일 쿼리
const xpathResults = corpus.xpath(
  '//sentence[entity[@type="ORG"]]/token[@pos="VERB"]'
);
```

### 1.4 직렬화 및 역직렬화

```typescript
// XML로 내보내기
const xmlString = data.toXML({
  pretty: true,
  indent: 2
});
console.log(xmlString);

// JSON으로 내보내기
const jsonString = data.toJSON({
  pretty: true,
  indent: 2
});

// CoNLL-U로 내보내기
const conlluString = sentence.toCoNLLU({
  includeComments: true,
  includeFeatures: true
});

// 파일로 저장
await data.saveToFile('corpus.xml', { format: 'xml' });
await data.saveToFile('corpus.json', { format: 'json' });
await data.saveToFile('corpus.conllu', { format: 'conllu' });

// 파일에서 로드
const loadedData = await LinguisticData.loadFromFile('corpus.xml');

// 문자열에서 파싱
const parsedData = LinguisticData.fromXML(xmlString);
const parsedJson = LinguisticData.fromJSON(jsonString);
```

### 1.5 검증

```typescript
import { Validator, ValidationLevel } from '@wia/lang-010';

const validator = new Validator({
  level: ValidationLevel.STRICT,
  schema: 'wia-lang-010-v1.0.xsd'
});

// 검증 실행
const result = await validator.validate(data);

if (result.isValid) {
  console.log('검증 성공!');
} else {
  console.log('검증 실패:');
  result.errors.forEach(error => {
    console.log(`  - ${error.message} at ${error.path}`);
  });

  result.warnings.forEach(warning => {
    console.log(`  ! ${warning.message}`);
  });
}

// 특정 제약 검증
const constraints = validator.checkConstraints(corpus, {
  morphology: [
    'pos_sequence_validity',
    'morpheme_alignment'
  ],
  syntax: [
    'single_root',
    'no_cycles',
    'projectivity'
  ],
  semantics: [
    'entity_span_validity',
    'entity_type_consistency'
  ]
});

// 품질 메트릭스
import { QualityMetrics } from '@wia/lang-010';

const metrics = new QualityMetrics(corpus);
const report = metrics.generate();

console.log(`문서 수: ${report.documentCount}`);
console.log(`문장 수: ${report.sentenceCount}`);
console.log(`토큰 수: ${report.tokenCount}`);
console.log(`평균 문장 길이: ${report.avgSentenceLength}`);
console.log(`어휘 다양성: ${report.lexicalDiversity}`);
console.log(`주석 밀도: ${report.annotationDensity}`);
```

### 1.6 변환 도구

```typescript
import { Converter } from '@wia/lang-010';

// Universal Dependencies에서 변환
const udData = `
# sent_id = 1
# text = The cat sits.
1	The	the	DET	DT	_	2	det	_	_
2	cat	cat	NOUN	NN	_	3	nsubj	_	_
3	sits	sit	VERB	VBZ	_	0	root	_	SpaceAfter=No
4	.	.	PUNCT	.	_	3	punct	_	_
`;

const wiaData = await Converter.fromCoNLLU(udData, {
  language: 'eng',
  preserveComments: true
});

// Penn Treebank에서 변환
const pennTree = '(S (NP (DT The) (NN cat)) (VP (VBZ sits)) (. .))';
const wiaFromPTB = await Converter.fromPennTreebank(pennTree, {
  language: 'eng'
});

// Tiger XML에서 변환
const tigerXML = await fs.readFile('tiger-corpus.xml', 'utf-8');
const wiaFromTiger = await Converter.fromTigerXML(tigerXML);

// 역방향 변환
const backToCoNLLU = await Converter.toCoNLLU(wiaData);
const backToPTB = await Converter.toPennTreebank(wiaData);
```

## 2. Python API

### 2.1 설치 및 기본 사용

```bash
# pip을 통한 설치
pip install wia-lang-010
```

```python
from wia_lang_010 import (
    LinguisticData,
    Corpus,
    Document,
    Sentence,
    Token,
    Metadata,
    Language
)

# 언어 데이터 생성
data = LinguisticData(
    version='1.0.0',
    metadata=Metadata(
        language=Language(
            iso639_3='kor',
            name='Korean',
            script='Hangul'
        ),
        title='My Korean Corpus',
        creator={
            'name': 'Research Lab',
            'email': 'lab@example.com'
        },
        license={
            'type': 'CC-BY-SA-4.0',
            'url': 'https://creativecommons.org/licenses/by-sa/4.0/'
        }
    )
)

# 코퍼스 구축
corpus = data.create_corpus()
doc = corpus.add_document(
    id='doc001',
    metadata={
        'source': '신문사 A',
        'date': '2025-01-20',
        'genre': 'news'
    }
)

sentence = doc.add_sentence(
    id='s001',
    text='인공지능 기술이 빠르게 발전하고 있다.'
)

# 토큰 추가
tokens = []
token_data = [
    ('t001', '인공지능', 0, 4),
    ('t002', '기술이', 5, 8),
    ('t003', '빠르게', 9, 12),
    ('t004', '발전하고', 13, 17),
    ('t005', '있다', 18, 20),
    ('t006', '.', 20, 21)
]

for tid, form, start, end in token_data:
    token = sentence.add_token(
        id=tid,
        form=form,
        start=start,
        end=end
    )
    tokens.append(token)

# 형태소 분석
tokens[0].add_morphemes([
    {'form': '인공', 'lemma': '인공', 'tag': 'NNG'},
    {'form': '지능', 'lemma': '지능', 'tag': 'NNG'}
])

# 품사 태깅
pos_tags = ['NOUN', 'NOUN', 'ADV', 'VERB', 'AUX', 'PUNCT']
for token, pos in zip(tokens, pos_tags):
    token.set_pos(pos)

# 의존 관계
dependencies = [
    (tokens[0], tokens[1], 'nmod'),
    (tokens[1], tokens[3], 'nsubj'),
    (tokens[2], tokens[3], 'advmod'),
    (tokens[3], tokens[4], 'aux'),
    (tokens[4], None, 'root'),
    (tokens[5], tokens[4], 'punct')
]

for token, head, rel in dependencies:
    token.set_dependency(head=head, relation=rel)
```

### 2.2 판다스 통합

```python
import pandas as pd
from wia_lang_010 import to_dataframe, from_dataframe

# 코퍼스를 DataFrame으로 변환
df = to_dataframe(corpus, level='token')

print(df.head())
# 출력:
#   doc_id  sent_id token_id     form    lemma   pos  xpos   head deprel
# 0 doc001     s001     t001  인공지능  인공지능  NOUN   NNG   t002   nmod
# 1 doc001     s001     t002    기술이     기술  NOUN   NNG   t004  nsubj
# 2 doc001     s001     t003    빠르게   빠르다   ADV    VA   t004 advmod
# ...

# DataFrame 조작
filtered = df[df['pos'] == 'NOUN']
grouped = df.groupby('pos').size()

# DataFrame에서 코퍼스 재구성
new_corpus = from_dataframe(df, format='token')

# 문장 수준 DataFrame
sent_df = to_dataframe(corpus, level='sentence')
print(sent_df.head())
# 출력:
#   doc_id  sent_id                               text  token_count
# 0 doc001     s001  인공지능 기술이 빠르게 발전하고 있다.            6
```

### 2.3 자연어 처리 파이프라인

```python
from wia_lang_010.pipeline import Pipeline, Tokenizer, POSTagger, Parser

# 파이프라인 구성
pipeline = Pipeline([
    Tokenizer(language='kor', tool='mecab'),
    POSTagger(model='sejong-tagset'),
    Parser(model='dependency', algorithm='biaffine')
])

# 텍스트 처리
text = "인공지능이 우리의 일상을 변화시키고 있다."
sentence = pipeline.process(text)

# 결과 확인
for token in sentence.tokens:
    print(f"{token.form}\t{token.pos}\t{token.head}\t{token.deprel}")

# 배치 처리
texts = [
    "첫 번째 문장입니다.",
    "두 번째 문장입니다.",
    "세 번째 문장입니다."
]

sentences = pipeline.process_batch(texts, batch_size=32)

# 사용자 정의 컴포넌트 추가
from wia_lang_010.pipeline import Component

class CustomNER(Component):
    def __init__(self, model_path):
        self.model = load_ner_model(model_path)

    def process(self, sentence):
        entities = self.model.predict(sentence.text)
        for ent in entities:
            sentence.add_entity(
                type=ent.type,
                start=ent.start,
                end=ent.end,
                confidence=ent.score
            )
        return sentence

pipeline.add(CustomNER('models/ner-korean.pkl'))
```

### 2.4 스트리밍 처리

```python
from wia_lang_010 import StreamingCorpus

# 대용량 코퍼스 스트리밍
corpus = StreamingCorpus.from_file('large-corpus.xml')

# 메모리 효율적인 반복
for document in corpus.iter_documents():
    for sentence in document.iter_sentences():
        # 문장별 처리
        process_sentence(sentence)

        # 메모리 해제
        del sentence
    del document

# 조건부 스트리밍
for sentence in corpus.iter_sentences(filter_func=lambda s: len(s.tokens) > 5):
    print(sentence.text)

# 배치 스트리밍
for batch in corpus.iter_batches(batch_size=1000):
    # 1000개 문장씩 처리
    results = process_batch(batch)
    save_results(results)
```

### 2.5 병렬 처리

```python
from wia_lang_010 import ParallelProcessor
from multiprocessing import Pool

# 병렬 처리 설정
processor = ParallelProcessor(
    num_workers=8,
    chunk_size=100
)

def analyze_sentence(sentence):
    # 무거운 분석 작업
    result = complex_analysis(sentence)
    return result

# 병렬 실행
results = processor.map(
    analyze_sentence,
    corpus.iter_sentences()
)

# 진행 상황 표시
from tqdm import tqdm

results = processor.map(
    analyze_sentence,
    tqdm(corpus.iter_sentences(), total=corpus.sentence_count)
)
```

## 3. Java API

### 3.1 Maven 설정

```xml
<dependency>
    <groupId>org.wia</groupId>
    <artifactId>wia-lang-010</artifactId>
    <version>1.0.0</version>
</dependency>
```

### 3.2 기본 사용

```java
import org.wia.lang010.*;
import org.wia.lang010.model.*;
import org.wia.lang010.io.*;

public class LinguisticDataExample {
    public static void main(String[] args) {
        // 메타데이터 생성
        Metadata metadata = Metadata.builder()
            .language(Language.builder()
                .iso639_3("kor")
                .name("Korean")
                .script("Hangul")
                .build())
            .title("My Korean Corpus")
            .creator(Creator.builder()
                .name("Research Lab")
                .email("lab@example.com")
                .build())
            .license(License.builder()
                .type("CC-BY-SA-4.0")
                .url("https://creativecommons.org/licenses/by-sa/4.0/")
                .build())
            .build();

        // 언어 데이터 생성
        LinguisticData data = new LinguisticData("1.0.0", metadata);

        // 코퍼스 구축
        Corpus corpus = data.createCorpus();
        Document doc = corpus.addDocument("doc001",
            Map.of("source", "신문사 A", "date", "2025-01-20"));

        Sentence sentence = doc.addSentence("s001",
            "인공지능 기술이 빠르게 발전하고 있다.");

        // 토큰 추가
        Token t1 = sentence.addToken("t001", "인공지능", 0, 4);
        Token t2 = sentence.addToken("t002", "기술이", 5, 8);
        Token t3 = sentence.addToken("t003", "빠르게", 9, 12);
        Token t4 = sentence.addToken("t004", "발전하고", 13, 17);
        Token t5 = sentence.addToken("t005", "있다", 18, 20);
        Token t6 = sentence.addToken("t006", ".", 20, 21);

        // 형태소 분석
        t1.addMorphemes(Arrays.asList(
            new Morpheme("인공", "인공", "NNG", "artificial"),
            new Morpheme("지능", "지능", "NNG", "intelligence")
        ));

        // 품사 태깅
        t1.setPos("NOUN", "NNG");
        t2.setPos("NOUN", "NNG+JKS");

        // 의존 관계
        t1.setDependency(t2, "nmod");
        t2.setDependency(t4, "nsubj");
        t3.setDependency(t4, "advmod");
        t4.setDependency(t5, "aux");
        t5.setDependency(null, "root");
        t6.setDependency(t5, "punct");

        // XML로 저장
        XMLWriter writer = new XMLWriter();
        writer.write(data, new File("corpus.xml"));

        // XML에서 로드
        XMLReader reader = new XMLReader();
        LinguisticData loaded = reader.read(new File("corpus.xml"));
    }
}
```

### 3.3 스트림 API

```java
import java.util.stream.*;

// 스트림을 사용한 쿼리
List<Token> nouns = corpus.getDocuments().stream()
    .flatMap(doc -> doc.getSentences().stream())
    .flatMap(sent -> sent.getTokens().stream())
    .filter(token -> "NOUN".equals(token.getPos()))
    .collect(Collectors.toList());

// 통계 계산
DoubleSummaryStatistics stats = corpus.getSentences().stream()
    .mapToDouble(sent -> sent.getTokens().size())
    .summaryStatistics();

System.out.println("평균 문장 길이: " + stats.getAverage());
System.out.println("최소 문장 길이: " + stats.getMin());
System.out.println("최대 문장 길이: " + stats.getMax());

// 병렬 스트림
Map<String, Long> posFrequency = corpus.getTokens().parallelStream()
    .collect(Collectors.groupingBy(
        Token::getPos,
        Collectors.counting()
    ));

// 필터링 및 변환
List<String> lemmas = corpus.getSentences().stream()
    .filter(sent -> sent.hasEntity("PERSON"))
    .flatMap(sent -> sent.getTokens().stream())
    .filter(token -> token.getLemma() != null)
    .map(Token::getLemma)
    .distinct()
    .sorted()
    .collect(Collectors.toList());
```

## 4. REST API

### 4.1 API 엔드포인트

```yaml
# WIA-LANG-010 REST API 명세

base_url: https://api.wia.org/lang-010/v1

endpoints:
  # 코퍼스 관리
  - path: /corpora
    methods:
      GET:
        description: 코퍼스 목록 조회
        parameters:
          - name: language
            type: string
            description: ISO 639-3 언어 코드
          - name: limit
            type: integer
            default: 20
          - name: offset
            type: integer
            default: 0
        response:
          type: array
          items: CorpusSummary

      POST:
        description: 새 코퍼스 생성
        request_body: LinguisticData
        response: CorpusCreated

  - path: /corpora/{corpus_id}
    methods:
      GET:
        description: 특정 코퍼스 조회
        response: LinguisticData

      PUT:
        description: 코퍼스 업데이트
        request_body: LinguisticData
        response: CorpusUpdated

      DELETE:
        description: 코퍼스 삭제
        response: CorpusDeleted

  # 검색
  - path: /search/tokens
    methods:
      POST:
        description: 토큰 검색
        request_body:
          corpus_id: string
          query:
            form: string (regex)
            pos: string
            lemma: string
        response:
          type: array
          items: Token

  - path: /search/sentences
    methods:
      POST:
        description: 문장 검색
        request_body:
          corpus_id: string
          query:
            text: string
            has_entity: string
            min_length: integer
        response:
          type: array
          items: Sentence

  # 주석 추가
  - path: /annotate/pos
    methods:
      POST:
        description: 자동 품사 태깅
        request_body:
          text: string
          language: string
        response:
          tokens: array

  - path: /annotate/parse
    methods:
      POST:
        description: 자동 구문 분석
        request_body:
          text: string
          language: string
          parser_type: constituency | dependency
        response:
          sentence: Sentence

  # 검증
  - path: /validate
    methods:
      POST:
        description: 데이터 검증
        request_body: LinguisticData
        response:
          is_valid: boolean
          errors: array
          warnings: array

  # 변환
  - path: /convert
    methods:
      POST:
        description: 형식 변환
        request_body:
          data: string
          from_format: xml | json | conllu
          to_format: xml | json | conllu
        response:
          converted_data: string
```

### 4.2 사용 예시

```javascript
// JavaScript/Node.js 클라이언트
const WiaLang010 = require('@wia/lang-010-client');

const client = new WiaLang010({
  apiKey: 'your-api-key',
  baseURL: 'https://api.wia.org/lang-010/v1'
});

// 코퍼스 조회
const corpora = await client.corpora.list({
  language: 'kor',
  limit: 10
});

// 새 코퍼스 생성
const newCorpus = await client.corpora.create({
  metadata: {
    language: { iso639_3: 'kor', name: 'Korean' },
    title: 'Test Corpus'
  }
});

// 자동 주석
const annotated = await client.annotate.pos({
  text: '인공지능 기술이 발전하고 있다.',
  language: 'kor'
});

// 토큰 검색
const tokens = await client.search.tokens({
  corpus_id: 'corpus-001',
  query: {
    pos: 'NOUN',
    form: '^[가-힣]+$'
  }
});
```

```python
# Python 클라이언트
from wia_lang_010_client import WiaLang010Client

client = WiaLang010Client(
    api_key='your-api-key',
    base_url='https://api.wia.org/lang-010/v1'
)

# 코퍼스 조회
corpora = client.corpora.list(language='kor', limit=10)

# 자동 구문 분석
result = client.annotate.parse(
    text='인공지능 기술이 발전하고 있다.',
    language='kor',
    parser_type='dependency'
)

# 검증
validation = client.validate(corpus_data)
if not validation['is_valid']:
    for error in validation['errors']:
        print(f"Error: {error['message']}")
```

## 5. GraphQL API

```graphql
# GraphQL 스키마
type Query {
  corpus(id: ID!): Corpus
  corpora(language: String, limit: Int, offset: Int): [Corpus!]!
  searchTokens(query: TokenQuery!): [Token!]!
  searchSentences(query: SentenceQuery!): [Sentence!]!
}

type Mutation {
  createCorpus(input: CreateCorpusInput!): Corpus!
  updateCorpus(id: ID!, input: UpdateCorpusInput!): Corpus!
  deleteCorpus(id: ID!): Boolean!

  addDocument(corpusId: ID!, input: DocumentInput!): Document!
  addSentence(documentId: ID!, input: SentenceInput!): Sentence!
  addToken(sentenceId: ID!, input: TokenInput!): Token!

  annotatePOS(text: String!, language: String!): [Token!]!
  annotateParse(text: String!, language: String!, type: ParserType!): Sentence!
}

type Corpus {
  id: ID!
  metadata: Metadata!
  documents: [Document!]!
  statistics: CorpusStatistics!
}

type Document {
  id: ID!
  metadata: DocumentMetadata
  sentences: [Sentence!]!
}

type Sentence {
  id: ID!
  text: String!
  tokens: [Token!]!
  entities: [Entity!]!
  dependencies: [Dependency!]!
}

type Token {
  id: ID!
  form: String!
  lemma: String
  pos: String
  xpos: String
  morphemes: [Morpheme!]
  head: Token
  deprel: String
}

input TokenQuery {
  corpusId: ID!
  form: String
  pos: String
  lemma: String
  hasHead: Boolean
}

input SentenceQuery {
  corpusId: ID!
  text: String
  hasEntity: String
  minLength: Int
  maxLength: Int
}
```

```javascript
// GraphQL 쿼리 예시
const query = `
  query GetCorpus($id: ID!) {
    corpus(id: $id) {
      id
      metadata {
        title
        language {
          name
          iso639_3
        }
      }
      statistics {
        documentCount
        sentenceCount
        tokenCount
      }
      documents {
        id
        sentences {
          text
          tokens {
            form
            pos
            lemma
          }
        }
      }
    }
  }
`;

const result = await client.query({
  query,
  variables: { id: 'corpus-001' }
});
```

## 결론

WIA-LANG-010 API는:

- **다양한 언어 지원**: TypeScript, Python, Java 등
- **유연한 인터페이스**: SDK, REST API, GraphQL
- **스트리밍 처리**: 대용량 데이터 효율적 처리
- **병렬 처리**: 멀티코어 활용
- **검증 및 품질 관리**: 자동화된 검증 도구
- **상호운용성**: 다른 형식과의 변환

다음 장에서는 언어 데이터 교환을 위한 프로토콜을 살펴보겠습니다.

---

**© 2025 SmileStory Inc. / WIA**
**弘益人間 (홍익인간) · Benefit All Humanity**

*"좋은 API는 복잡한 작업을 간단하게 만들고, 위대한 API는 불가능해 보이던 일을 가능하게 합니다."*
